


// --- CRSF Protocol Implementation Functions ---

/*
 * @brief CRC-8 with polynomial 0xD5 (used by CRSF)
 * @param data Pointer to the data for CRC calculation.
 * @param len The length of the data.
 * @return The calculated CRC-8 value.
 */
uint8_t crc8_d5(const uint8_t *data, uint8_t len) {
  uint8_t crc = 0;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x80)
        crc = (crc << 1) ^ 0xD5;
      else
        crc <<= 1;
    }
  }
  return crc;
}

/*
 * @brief Unpacks 16 RC channels (11 bits each) from a byte stream.
 * @param data Pointer to the packed channel data.
 * @param out Pointer to the output array for the 16 channel values.
 */
void unpackCh(const uint8_t *data, uint16_t *out) {
  uint32_t bitBuffer = 0;
  int bitCount = 0;
  int outIndex = 0;

  for (int i = 0; i < 22; i++) {
    bitBuffer |= ((uint32_t)data[i]) << bitCount;
    bitCount += 8;

    while (bitCount >= 11) {
      out[outIndex++] = bitBuffer & 0x7FF; // extract 11 bits (0x7FF = 2047)
      bitBuffer >>= 11;
      bitCount -= 11;
    }
  }
}