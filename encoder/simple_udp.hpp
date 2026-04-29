#pragma once

#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <cstdint>
#include <string>

class simple_udp_client
{
	int sockfd;
	sockaddr_in dst_addr;

public:
	simple_udp_client(const std::string &host, int port) : sockfd(-1)
	{
		sockfd = socket(AF_INET, SOCK_DGRAM, 0);
		memset(&dst_addr, 0, sizeof(dst_addr));
		dst_addr.sin_family = AF_INET;
		dst_addr.sin_addr.s_addr = inet_addr(host.c_str());
		dst_addr.sin_port = htons(port);
		// Larger send buffer keeps full-frame bursts from blocking.
		int sndbuf = 8 * 1024 * 1024;
		setsockopt(sockfd, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));
	}

	~simple_udp_client() { destroy(); }

	void destroy()
	{
		if (sockfd >= 0)
		{
			close(sockfd);
			sockfd = -1;
		}
	}

	bool is_open() const { return sockfd >= 0; }

	bool sendto_one(const uint8_t *data, size_t len)
	{
		if (sockfd < 0)
			return false;
		ssize_t n = sendto(sockfd, data, len, 0, reinterpret_cast<sockaddr *>(&dst_addr),
						   sizeof(dst_addr));
		return n == static_cast<ssize_t>(len);
	}
};
