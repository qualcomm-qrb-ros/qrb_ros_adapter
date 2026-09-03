// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_adapter/ipc_runtime/ipc_endpoint.hpp"

#include "qrb_ros_adapter/ipc_runtime/transport_codec.hpp"

#include <cerrno>
#include <cstring>
#include <string>

#include <poll.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>

namespace qrb::adapter
{

namespace
{

bool MakeSockaddr( const std::string &path, sockaddr_un &addr )
{
    if ( path.empty() || path.size() >= sizeof( addr.sun_path ) ) return false;
    std::memset( &addr, 0, sizeof( addr ) );
    addr.sun_family = AF_UNIX;
    std::memcpy( addr.sun_path, path.c_str(), path.size() + 1 );
    return true;
}

Status ValidateParentDirectory( const std::string &path )
{
    std::size_t slash = path.find_last_of( '/' );
    if ( slash == std::string::npos ) return Status::kOk;

    const std::string parent = slash == 0 ? "/" : path.substr( 0, slash );
    struct stat info{};
    if ( ::stat( parent.c_str(), &info ) != 0 || !S_ISDIR( info.st_mode ) )
    {
        return Status::kResourceError;
    }
    return Status::kOk;
}

Status RemoveStaleSocket( const std::string &path )
{
    struct stat info{};
    if ( ::lstat( path.c_str(), &info ) != 0 )
    {
        return errno == ENOENT ? Status::kOk : Status::kResourceError;
    }
    if ( !S_ISSOCK( info.st_mode ) || info.st_uid != ::geteuid() )
    {
        return Status::kResourceError;
    }

    sockaddr_un addr{};
    if ( !MakeSockaddr( path, addr ) ) return Status::kInvalidArgument;
    const int probe = ::socket( AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC, 0 );
    if ( probe < 0 ) return Status::kResourceError;

    const int connected =
        ::connect( probe, reinterpret_cast<sockaddr *>( &addr ), sizeof( addr ) );
    const int probeError = connected == 0 ? 0 : errno;
    ::close( probe );
    if ( connected == 0 || probeError != ECONNREFUSED ) return Status::kResourceError;

    if ( ::unlink( path.c_str() ) != 0 && errno != ENOENT ) return Status::kResourceError;
    return Status::kOk;
}

/** @brief poll() one fd for POLLIN, translating the outcome to Status. */
Status WaitReadable( int fd, std::chrono::milliseconds timeout )
{
    pollfd pfd{ fd, POLLIN, 0 };
    int rc = ::poll( &pfd, 1, static_cast<int>( timeout.count() ) );
    if ( rc == 0 ) return Status::kTimeout;
    if ( rc < 0 ) return ( errno == EINTR ) ? Status::kTimeout : Status::kResourceError;

    // POLLHUP (and even POLLERR) can be set *together* with POLLIN when the
    // peer has already closed its end but a frame it sent earlier is still
    // sitting unread in this socket's receive buffer -- exactly the
    // CONNECT_RESPONSE-then-Close() reject path, or any SendFrame()
    // immediately followed by Stop(). Treating a hangup as fatal before
    // checking POLLIN would silently discard that frame. Only a hangup with
    // *nothing* left to read is an error: the next RecvFrame() gets a clean
    // EOF via recvmsg()==0 once the buffer really is drained.
    if ( pfd.revents & POLLIN ) return Status::kOk;
    if ( pfd.revents & ( POLLERR | POLLHUP | POLLNVAL ) ) return Status::kResourceError;
    return Status::kResourceError;
}

}   // namespace

IpcEndpoint::~IpcEndpoint() { Close(); }

Status IpcEndpoint::Listen( const std::string &socketPath )
{
    if ( fd_ >= 0 || listenFd_ >= 0 ) return Status::kBadState;

    sockaddr_un addr{};
    if ( !MakeSockaddr( socketPath, addr ) ) return Status::kInvalidArgument;

    Status parent = ValidateParentDirectory( socketPath );
    if ( parent != Status::kOk ) return parent;
    Status stale = RemoveStaleSocket( socketPath );
    if ( stale != Status::kOk ) return stale;

    int fd = ::socket( AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC, 0 );
    if ( fd < 0 ) return Status::kResourceError;

    if ( ::bind( fd, reinterpret_cast<sockaddr *>( &addr ), sizeof( addr ) ) != 0 )
    {
        ::close( fd );
        return Status::kResourceError;
    }

    if ( ::chmod( socketPath.c_str(), 0660 ) != 0 )
    {
        ::close( fd );
        ::unlink( socketPath.c_str() );
        return Status::kResourceError;
    }

    // Backlog of 1: one Flow, one peer (HLD §4.3).
    if ( ::listen( fd, 1 ) != 0 )
    {
        ::close( fd );
        ::unlink( socketPath.c_str() );
        return Status::kResourceError;
    }

    listenFd_ = fd;
    socketPath_ = socketPath;
    isServer_ = true;
    return Status::kOk;
}

Status IpcEndpoint::Accept( std::chrono::milliseconds timeout )
{
    bool interrupted = false;
    int connection = -1;
    Status accepted = WaitAndAccept( timeout, -1, interrupted, connection );
    if ( accepted != Status::kOk ) return accepted;

    // One Flow, one peer: nothing remains to listen for after acceptance.
    ::close( listenFd_ );
    listenFd_ = -1;
    fd_ = connection;
    return Status::kOk;
}

Status IpcEndpoint::WaitAndAccept(
    std::chrono::milliseconds timeout, int wakeFd, bool &woken, int &connOut )
{
    woken = false;
    connOut = -1;
    if ( listenFd_ < 0 ) return Status::kBadState;

    pollfd pollFds[2] = { { listenFd_, POLLIN, 0 }, { wakeFd, POLLIN, 0 } };
    const nfds_t count = wakeFd >= 0 ? 2 : 1;
    int rc = ::poll( pollFds, count, static_cast<int>( timeout.count() ) );
    if ( rc == 0 ) return Status::kTimeout;
    if ( rc < 0 ) return errno == EINTR ? Status::kTimeout : Status::kResourceError;

    if ( wakeFd >= 0 && pollFds[1].revents & POLLIN )
    {
        woken = true;
        return Status::kOk;
    }
    if ( !( pollFds[0].revents & POLLIN ) ) return Status::kResourceError;

    connOut = ::accept4( listenFd_, nullptr, nullptr, SOCK_CLOEXEC );
    if ( connOut < 0 ) return Status::kResourceError;
    return Status::kOk;
}

Status IpcEndpoint::AcceptConnection( IpcEndpoint &connectionOut,
    std::chrono::milliseconds timeout, int wakeFd, bool &interrupted )
{
    if ( &connectionOut == this ) return Status::kInvalidArgument;
    if ( connectionOut.fd_ >= 0 || connectionOut.listenFd_ >= 0 ) return Status::kBadState;

    int conn = -1;
    Status accepted = WaitAndAccept( timeout, wakeFd, interrupted, conn );
    if ( accepted != Status::kOk ) return accepted;

    connectionOut.fd_ = conn;
    connectionOut.isServer_ = false;
    return Status::kOk;
}

Status IpcEndpoint::ConnectOnce( const std::string &socketPath )
{
    if ( fd_ >= 0 ) return Status::kBadState;

    sockaddr_un addr{};
    if ( !MakeSockaddr( socketPath, addr ) ) return Status::kInvalidArgument;

    int fd = ::socket( AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC, 0 );
    if ( fd < 0 ) return Status::kResourceError;

    if ( ::connect( fd, reinterpret_cast<sockaddr *>( &addr ), sizeof( addr ) ) == 0 )
    {
        fd_ = fd;
        socketPath_ = socketPath;
        isServer_ = false;
        return Status::kOk;
    }

    int err = errno;
    ::close( fd );

    // The peer has not bound or is not accepting yet. Expose that distinction
    // so a caller can keep waiting within one bounded startup operation. Once a
    // Flow has failed or disconnected, its lifecycle does not call ConnectOnce()
    // again.
    if ( err == ENOENT || err == ECONNREFUSED || err == EAGAIN || err == EINTR )
    {
        return Status::kTimeout;
    }
    return Status::kResourceError;
}

Status IpcEndpoint::SendFrame( Operation operation, std::uint64_t requestId, const void *payload,
    std::size_t payloadSize, const int *fds, std::uint32_t fdCount, std::uint32_t flags,
    std::uint64_t payloadOffset )
{
    if ( fd_ < 0 ) return Status::kBadState;
    if ( payloadSize > kMaxPayloadSize ) return Status::kInvalidArgument;
    if ( fdCount > kMaxFdCount ) return Status::kInvalidArgument;
    if ( fdCount > 0 && fds == nullptr ) return Status::kInvalidArgument;
    if ( fdCount == 0 && payloadOffset != 0 ) return Status::kInvalidArgument;
    if ( payloadSize > 0 && payload == nullptr ) return Status::kInvalidArgument;

    TransportHeader header;
    header.operation = static_cast<std::uint16_t>( operation );
    header.payloadSize = static_cast<std::uint32_t>( payloadSize );
    header.requestId = requestId;
    header.flags = flags;
    header.fdCount = fdCount;
    header.payloadOffset = payloadOffset;

    std::vector<std::uint8_t> headerBytes = EncodeTransportHeader( header );

    iovec iov[2];
    iov[0].iov_base = headerBytes.data();
    iov[0].iov_len = headerBytes.size();
    int iovCount = 1;
    if ( payloadSize > 0 )
    {
        iov[1].iov_base = const_cast<void *>( payload );
        iov[1].iov_len = payloadSize;
        iovCount = 2;
    }

    msghdr msg{};
    msg.msg_iov = iov;
    msg.msg_iovlen = static_cast<std::size_t>( iovCount );

    alignas( cmsghdr ) char cmsgBuf[CMSG_SPACE( sizeof( int ) * kMaxFdCount )];
    if ( fdCount > 0 )
    {
        std::memset( cmsgBuf, 0, sizeof( cmsgBuf ) );
        msg.msg_control = cmsgBuf;
        msg.msg_controllen = CMSG_SPACE( sizeof( int ) * fdCount );
        cmsghdr *cmsg = CMSG_FIRSTHDR( &msg );
        cmsg->cmsg_level = SOL_SOCKET;
        cmsg->cmsg_type = SCM_RIGHTS;
        cmsg->cmsg_len = CMSG_LEN( sizeof( int ) * fdCount );
        std::memcpy( CMSG_DATA( cmsg ), fds, sizeof( int ) * fdCount );
        msg.msg_controllen = cmsg->cmsg_len;
    }

    ssize_t sent = ::sendmsg( fd_, &msg, MSG_DONTWAIT | MSG_NOSIGNAL );
    if ( sent < 0 ) return Status::kResourceError;   // includes EAGAIN -- see header doc
    return Status::kOk;
}

Status IpcEndpoint::RecvFrame( TransportHeader &headerOut, std::vector<std::uint8_t> &payloadOut,
    std::vector<OwnedFd> &fdsOut, std::chrono::milliseconds timeout )
{
    fdsOut.clear();
    if ( fd_ < 0 ) return Status::kBadState;

    Status wait = WaitReadable( fd_, timeout );
    if ( wait != Status::kOk ) return wait;

    const std::size_t receiveCapacity = kTransportHeaderEncodedSize + kMaxPayloadSize;
    if ( receiveBuffer_.size() != receiveCapacity ) receiveBuffer_.resize( receiveCapacity );
    iovec iov{ receiveBuffer_.data(), receiveBuffer_.size() };

    alignas( cmsghdr ) char cmsgBuf[CMSG_SPACE( sizeof( int ) * kMaxFdCount )];
    std::memset( cmsgBuf, 0, sizeof( cmsgBuf ) );

    msghdr msg{};
    msg.msg_iov = &iov;
    msg.msg_iovlen = 1;
    msg.msg_control = cmsgBuf;
    msg.msg_controllen = sizeof( cmsgBuf );

    ssize_t received = ::recvmsg( fd_, &msg, MSG_CMSG_CLOEXEC );
    if ( received < 0 ) return Status::kResourceError;
    if ( received == 0 ) return Status::kResourceError;   // peer shut down cleanly -- EOF

    // Take ownership of every fd the kernel installed *before* any validation
    // below can return early -- otherwise a malformed frame that happened to
    // carry fds would leak them.
    for ( cmsghdr *cmsg = CMSG_FIRSTHDR( &msg ); cmsg != nullptr; cmsg = CMSG_NXTHDR( &msg, cmsg ) )
    {
        if ( cmsg->cmsg_level != SOL_SOCKET || cmsg->cmsg_type != SCM_RIGHTS ) continue;
        if ( cmsg->cmsg_len < CMSG_LEN( 0 ) ) continue;

        std::size_t bytes = cmsg->cmsg_len - CMSG_LEN( 0 );
        std::size_t count = bytes / sizeof( int );
        for ( std::size_t i = 0; i < count; ++i )
        {
            int fd = -1;
            std::memcpy( &fd, CMSG_DATA( cmsg ) + i * sizeof( int ), sizeof( int ) );
            fdsOut.emplace_back( fd );
        }
    }

    // Control data did not fit: some fds were dropped by the kernel and we
    // cannot tell which, so the frame is unusable.
    if ( msg.msg_flags & MSG_CTRUNC )
    {
        fdsOut.clear();
        return Status::kProtocolError;
    }
    if ( msg.msg_flags & MSG_TRUNC )   // datagram larger than our buffer
    {
        fdsOut.clear();
        return Status::kProtocolError;
    }

    if ( static_cast<std::size_t>( received ) < kTransportHeaderEncodedSize )
    {
        fdsOut.clear();
        return Status::kProtocolError;
    }

    Status decoded = DecodeTransportHeader(
        receiveBuffer_.data(), kTransportHeaderEncodedSize, headerOut );
    if ( decoded != Status::kOk )
    {
        fdsOut.clear();
        return decoded;
    }

    std::size_t actualPayload = static_cast<std::size_t>( received ) - kTransportHeaderEncodedSize;
    if ( actualPayload != headerOut.payloadSize )
    {
        fdsOut.clear();
        return Status::kProtocolError;
    }

    // The header's fd count is part of the contract, so a peer whose
    // SCM_RIGHTS block disagrees with it is speaking a different protocol.
    if ( fdsOut.size() != headerOut.fdCount )
    {
        fdsOut.clear();
        return Status::kProtocolError;
    }

    payloadOut.assign( receiveBuffer_.begin()
            + static_cast<std::ptrdiff_t>( kTransportHeaderEncodedSize ),
        receiveBuffer_.begin()
            + static_cast<std::ptrdiff_t>( kTransportHeaderEncodedSize + actualPayload ) );
    return Status::kOk;
}

void IpcEndpoint::Close() noexcept
{
    if ( fd_ >= 0 )
    {
        ::close( fd_ );
        fd_ = -1;
    }
    if ( listenFd_ >= 0 )
    {
        ::close( listenFd_ );
        listenFd_ = -1;
    }
    if ( isServer_ && !socketPath_.empty() )
    {
        ::unlink( socketPath_.c_str() );
        socketPath_.clear();
    }
}

}   // namespace qrb::adapter
