#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "master_board_sdk/ETHERNET_types.h"

#define max(a,b) (a>b ? a : b)


void ETHERNET_packet::set_src_mac(uint8_t my_mac[6]) {
	memcpy(this->data.src_mac, my_mac, sizeof(uint8_t)*6);
}

void ETHERNET_packet::set_dst_mac(uint8_t dst_mac[6]) {
	memcpy(this->data.dst_mac, dst_mac, sizeof(uint8_t)*6);
}

int ETHERNET_packet::toBytes(uint8_t *bytes, int max_len) {	
	int correct_len = static_cast<int>(sizeof(ETHERNET_data)) + this->data.length - 0xff;
	int padded_len = max(correct_len, ETH_SEND_SIZE_MIN);

#ifdef DEBUG
	assert(padded_len <= max_len);
#else
	if (padded_len > max_len) printf("padded_len > max_len - should not happen");
#endif

	memcpy(bytes, &(this->data), (uint8_t)correct_len);
	
	if(padded_len > correct_len) {
		memset(bytes + correct_len, 0, (uint8_t)(padded_len - correct_len));
	}

	return padded_len;
}

uint8_t* ETHERNET_packet::get_payload_ptr() {
	return this->data.payload;
}

int ETHERNET_packet::get_payload_len() {
	return this->data.length;
}

void ETHERNET_packet::set_payload_len(int len) {
	this->data.length = static_cast<uint16_t>(len);
}

uint8_t* ETHERNET_packet::get_src_mac_FromRaw(uint8_t *raw_bytes, int len) {
	if(len < ETH_RECV_SIZE_MIN) return NULL;

	return raw_bytes + 6;
}

int ETHERNET_packet::get_payload_len_FromRaw(uint8_t *raw_bytes, int len) {
	if(len < ETH_RECV_SIZE_MIN) return -1;

	return ((ETHERNET_data *) raw_bytes)->length;
}

uint8_t* ETHERNET_packet::get_payload_FromRaw(uint8_t *raw_bytes, int len) {
	if(len < ETH_RECV_SIZE_MIN + 1) return NULL;
	
	return ((ETHERNET_data *) raw_bytes)->payload;
}