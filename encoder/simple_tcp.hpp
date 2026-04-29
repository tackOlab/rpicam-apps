#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

class simple_tcp
{
	int sockfd;
	int client_sockfd;
	sockaddr_in addr;
	sockaddr_in from_addr;
	bool is_server_;

public:
	simple_tcp(std::string address, int port) : sockfd(-1), client_sockfd(-1), is_server_(false)
	{
		addr.sin_family = AF_INET;
		addr.sin_addr.s_addr = inet_addr(address.c_str());
		addr.sin_port = htons(port);
	}

	~simple_tcp()
	{
		this->destroy();
	}

	void destroy()
	{
		if (client_sockfd >= 0)
		{
			close(client_sockfd);
			client_sockfd = -1;
		}
		if (sockfd >= 0)
		{
			close(sockfd);
			sockfd = -1;
		}
	}

	void disconnect_client()
	{
		if (client_sockfd >= 0)
		{
			close(client_sockfd);
			client_sockfd = -1;
		}
	}

	int bind_listen()
	{
		is_server_ = true;
		sockfd = socket(AF_INET, SOCK_STREAM, 0);
		if (sockfd < 0)
			return -1;
		int opt = 1;
		setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt));
		if (bind(sockfd, (struct sockaddr *)&addr, sizeof(addr)) != 0)
			return -1;
		if (listen(sockfd, SOMAXCONN) != 0)
			return -1;
		return 0;
	}

	int accept_one()
	{
		if (client_sockfd >= 0)
		{
			close(client_sockfd);
			client_sockfd = -1;
		}
		socklen_t len = sizeof(sockaddr_in);
		client_sockfd = accept(sockfd, (struct sockaddr *)&from_addr, &len);
		return client_sockfd >= 0 ? 0 : -1;
	}

	int create_server()
	{
		if (bind_listen() != 0)
			return -1;
		return accept_one() == 0 ? sockfd : -1;
	}

	int create_client()
	{
		is_server_ = false;
		sockfd = socket(AF_INET, SOCK_STREAM, 0);
		if (sockfd < 0)
			return -1;
		int nodelay = 1;
		setsockopt(sockfd, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));
		int keepalive = 1;
		setsockopt(sockfd, SOL_SOCKET, SO_KEEPALIVE, &keepalive, sizeof(keepalive));
		if (connect(sockfd, (struct sockaddr *)&addr, sizeof(addr)) != 0)
		{
			close(sockfd);
			sockfd = -1;
			return -1;
		}
		return 0;
	}

	bool is_connected() const
	{
		return is_server_ ? client_sockfd >= 0 : sockfd >= 0;
	}

	int active_fd() const
	{
		return is_server_ ? client_sockfd : sockfd;
	}

	bool send_all(const uint8_t *src, size_t len)
	{
		int fd = active_fd();
		if (fd < 0)
			return false;
		size_t sent = 0;
		while (sent < len)
		{
			ssize_t n = send(fd, src + sent, len - sent, MSG_NOSIGNAL);
			if (n <= 0)
				return false;
			sent += static_cast<size_t>(n);
		}
		return true;
	}

	bool recv_all(uint8_t *dst, size_t len)
	{
		int fd = active_fd();
		if (fd < 0)
			return false;
		size_t got = 0;
		while (got < len)
		{
			ssize_t n = recv(fd, dst + got, len - got, 0);
			if (n <= 0)
				return false;
			got += static_cast<size_t>(n);
		}
		return true;
	}

	bool SendFramed(const uint8_t *src, uint32_t len)
	{
		uint32_t len_be = htonl(len);
		if (!send_all(reinterpret_cast<const uint8_t *>(&len_be), 4))
			return false;
		return send_all(src, len);
	}

	bool RecvFramed(std::vector<uint8_t> &dst, uint32_t &len)
	{
		uint32_t len_be = 0;
		if (!recv_all(reinterpret_cast<uint8_t *>(&len_be), 4))
			return false;
		len = ntohl(len_be);
		if (len > 64u * 1024u * 1024u)
			return false;
		if (dst.size() < len)
			dst.resize(len);
		return recv_all(dst.data(), len);
	}

	int Rx(uint8_t *dst)
	{
		std::vector<uint8_t> buf(5000000);
		int len = 0;
		int fd = active_fd();
		if (fd < 0)
			return 0;
		int rsize = recv(fd, buf.data(), buf.size(), 0);
		len += rsize;
		while (rsize > 0)
		{
			for (int i = 0; i < rsize; ++i)
			{
				dst[i] = buf[i];
			}
			dst += rsize;
			rsize = recv(fd, buf.data(), buf.size(), 0);
			len += rsize;
		}
		return len;
	}

	void Tx(uint8_t *src, size_t len)
	{
		send_all(src, len);
	}
};
