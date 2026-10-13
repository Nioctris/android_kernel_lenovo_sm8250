// SPDX-License-Identifier: GPL-2.0
/*
 * Goodix TEE fingerprint driver, Lenovo TB371FC variant: netlink events
 *
 * Copyright (c) 2017 Goodix
 */
#define pr_fmt(fmt) "Goodixfp " fmt

#include <linux/module.h>
#include <linux/types.h>
#include <net/netlink.h>
#include <net/sock.h>

#include "gf_spi.h"

#define MAX_MSGSIZE 32

static int pid = -1;
static struct sock *nl_sk;

int sendnlmsg(char *msg)
{
	struct sk_buff *skb;
	struct nlmsghdr *nlh;
	int len = NLMSG_SPACE(MAX_MSGSIZE);
	int ret;

	if (!msg || !nl_sk || !pid)
		return -ENODEV;

	skb = alloc_skb(len, GFP_ATOMIC);
	if (!skb)
		return -ENOMEM;

	nlh = nlmsg_put(skb, 0, 0, 0, MAX_MSGSIZE, 0);
	if (!nlh) {
		kfree_skb(skb);
		return -EMSGSIZE;
	}

	NETLINK_CB(skb).portid = 0;
	NETLINK_CB(skb).dst_group = 0;

	memcpy(NLMSG_DATA(nlh), msg, sizeof(char));
	pr_debug("send message: %d\n", *(char *)NLMSG_DATA(nlh));

	ret = netlink_unicast(nl_sk, skb, pid, MSG_DONTWAIT);
	if (ret > 0)
		ret = 0;

	return ret;
}

/* The HAL announces itself with any message; remember its port id. */
static void nl_data_ready(struct sk_buff *skb)
{
	struct nlmsghdr *nlh;

	if (skb->len < NLMSG_SPACE(0))
		return;

	nlh = nlmsg_hdr(skb);
	pid = nlh->nlmsg_pid;
}

int netlink_init(void)
{
	struct netlink_kernel_cfg netlink_cfg = {
		.input = nl_data_ready,
	};

	nl_sk = netlink_kernel_create(&init_net, NETLINK_TEST, &netlink_cfg);
	if (!nl_sk) {
		pr_err("create netlink socket error\n");
		return -ENOMEM;
	}

	return 0;
}

void netlink_exit(void)
{
	if (nl_sk) {
		netlink_kernel_release(nl_sk);
		nl_sk = NULL;
	}
}
