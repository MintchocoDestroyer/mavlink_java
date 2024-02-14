package io.dronefleet.mavlink.protocol.util;

import java.nio.charset.StandardCharsets;

public class CrcX25 {
    private int crc;

    public CrcX25() {
        crc = 0xffff;
    }   //crc는 메세지 번호이다. 4바이트를 일부러 f로 마스킹해 연산을 위한 초기 상태를 만든다

    public void accumulate(String str) {
        accumulate(str.getBytes(StandardCharsets.UTF_8));
    }

    public void accumulate(byte[] bytes) {
        accumulate(bytes, 0, bytes.length);
    }   //바이트를 매개변수에 길이 저장

    public void accumulate(byte[] bytes, int offset, int length) {
        for (int i = offset; i < length; i++) {
            accumulate(bytes[i]);
        }
    }

    public void accumulate(int b) {
        b = b ^ (crc & 0xff);
        b ^= (b << 4) & 0xff;
        b &= 0xff;
        crc = (crc >> 8) ^ (b << 8) ^ (b << 3) ^ (b >> 4);
    }

    public int get() {
        return crc & 0xffff;
        }   //메세지 기능이 한 번 돌면 다시 마스킹하여 초기 상태로 돌아감
    }
