/*************************************************************************/ /*
 avb-mse

 Copyright (C) 2015-2018 Renesas Electronics Corporation

 License        Dual MIT/GPLv2

 The contents of this file are subject to the MIT license as set out below.

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.

 Alternatively, the contents of this file may be used under the terms of
 the GNU General Public License Version 2 ("GPL") in which case the provisions
 of GPL are applicable instead of those above.

 If you wish to allow use of your version of this file only under the terms of
 GPL, and not to allow others to use your version of this file under the terms
 of the MIT license, indicate your decision by deleting the provisions above
 and replace them with the notice and other provisions required by GPL as set
 out in the file called "GPL-COPYING" included in this distribution. If you do
 not delete the provisions above, a recipient may use your version of this file
 under the terms of either the MIT license or GPL.

 This License is also included in this distribution in the file called
 "MIT-COPYING".

 EXCEPT AS OTHERWISE STATED IN A NEGOTIATED AGREEMENT: (A) THE SOFTWARE IS
 PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
 BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
 PURPOSE AND NONINFRINGEMENT; AND (B) IN NO EVENT SHALL THE AUTHORS OR
 COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.


 GPLv2:
 If you wish to use this file under the terms of GPL, following terms are
 effective.

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; version 2 of the License.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/ /*************************************************************************/

#ifndef __MSE_PACKET_CTRL_H__
#define __MSE_PACKET_CTRL_H__

struct mse_packet_ctrl {
	struct device *dev;
	int size;
	int write_p;
	int read_p;
	int max_packet_size;
	dma_addr_t dma_handle;
	void *dma_vaddr;
	struct mse_packet *packet_table;
};

int mse_packet_ctrl_check_packet_remain(struct mse_packet_ctrl *dma);
struct mse_packet_ctrl *mse_packet_ctrl_alloc(struct device *dev,
					      int max_packet,
					      int max_packet_size);
void mse_packet_ctrl_free(struct mse_packet_ctrl *dma);
int mse_packet_ctrl_make_packet(int index,
				void *data,
				size_t size,
				int ptp_clock,
				int *current_timestamp,
				int timestamps_size,
				u64 *timestamps,
				struct mse_packet_ctrl *dma,
				struct mse_packetizer_ops *ops,
				size_t *processed);
int mse_packet_ctrl_send_prepare_packet(int index,
					struct mse_packet_ctrl *dma,
					struct mse_adapter_network_ops *ops);
int mse_packet_ctrl_send_packet(int index,
				struct mse_packet_ctrl *dma,
				struct mse_adapter_network_ops *ops);
int mse_packet_ctrl_receive_prepare_packet(int index,
					   struct mse_packet_ctrl *dma,
					   struct mse_adapter_network_ops *ops);
int mse_packet_ctrl_receive_packet(int index,
				   int max_size,
				   struct mse_packet_ctrl *dma,
				   struct mse_adapter_network_ops *ops);
int mse_packet_ctrl_receive_packet_crf(int index,
				       int max_size,
				       struct mse_packet_ctrl *dma,
				       struct mse_adapter_network_ops *ops);
int mse_packet_ctrl_take_out_packet(int index,
				    void *data,
				    size_t size,
				    u64 *timestamps,
				    int timestamps_size,
				    int *timestamps_stored,
				    struct mse_packet_ctrl *dma,
				    struct mse_packetizer_ops *ops,
				    size_t *processed);
int mse_packet_ctrl_make_packet_crf(int index,
				    u64 *timestamps,
				    int timestamps_size,
				    struct mse_packet_ctrl *dma);
int mse_packet_ctrl_take_out_packet_crf(int index,
					u64 *timestamps,
					int timestamps_size,
					struct mse_packet_ctrl *dma);
void mse_packet_ctrl_discard_packet(struct mse_packet_ctrl *dma);

#endif /* __MSE_PACKET_CTRL_H__ */
