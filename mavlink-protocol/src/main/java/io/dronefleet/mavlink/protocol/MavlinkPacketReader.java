package io.dronefleet.mavlink.protocol;

import java.io.EOFException;
import java.io.IOException;
import java.io.InputStream;

/**
 * <p>
 *  {@link InputStream} 에서 Mavlink 프로토콜 패킷 읽기.
 * <p>
 * 이 클래스에서 읽은 패킷은 CRC 검사를 받지 않았으므로 유효하지 않을 수 있습니다. 이 클래스의 사용자
 * {@link MavlinkPacket#validateCrc(int) validate packet CRC} 및 {@link #drop() drop}을(를) 확인해야 합니다
 * 유효성 검사를 통과하지 못하는 패킷. 이렇게 하면 유효하지 않은 패킷이 처리되지 않습니다,
 * 그리고 유효한 패킷은 유효하지 않은 패킷으로 건너뛰지 않습니다.
 */
public class MavlinkPacketReader {
    private final MavlinkFrameReader in;

    /**
     * {@link InputStream}에 대한 새 판독기를 구성합니다
     *
     * @param in 으로 부터 입력 스트림을 읽음.
     */
    public MavlinkPacketReader(InputStream in) {
        this.in = new MavlinkFrameReader(in);
    }

    /**
     * 스트림에서 다음 패킷 읽기.
     *
     * @throws IOException  IO 에러가 발생 시.
     * @throws EOFException 스트림 끝에서 에러 발생 시.
     */
    public MavlinkPacket next() throws IOException {
        while (in.next()) {
            byte[] frame = in.frame();
            switch (frame[0] & 0xff) {
                case MavlinkPacket.MAGIC_V1:
                    return MavlinkPacket.fromV1Bytes(frame);
                case MavlinkPacket.MAGIC_V2:
                    return MavlinkPacket.fromV2Bytes(frame);
            }

            // 프레임이 우리가 알아들을 수 있는 매직 마커로 시작되지 않았을 때.
            in.drop();
        }
        throw new EOFException("End of stream");
    }

    /**
     * 마지막 읽기 패킷을 드랍하여 첫 번째 바이트를 건너뛰고 해당 바이트를 스트림으로 반환합니다.
     *
     * @throws IOException IO 에러 발생 시.
     */
    public void drop() throws IOException {
        in.drop();
    }
}
