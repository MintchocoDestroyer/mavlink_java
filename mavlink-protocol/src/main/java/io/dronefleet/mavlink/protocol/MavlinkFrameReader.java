package io.dronefleet.mavlink.protocol;

import java.io.IOException;
import java.io.InputStream;

/**
 * <p>
 * Reads Mavlink protocol frames.
 * <p>
 * A frame is a collection of bytes that appear to be a complete packet. A frame begins with
 * a valid protocol version marker (STX), and includes the rest of the expected packet bytes
 * according to the specified length, protocol version, and incompatibility flags (signature,
 * for instance).
 * <p>
 * The parsed frames are unreliable in the sense that the only actual test is that the frame
 * began with a valid STX. Any long enough sequence of bytes which begins a valid STX will be
 * returned as a frame by this reader. Therefore, users of this class are required to understand
 * and CRC check the returned frames, issuing calls to {@link #drop()} when validation fails.
 */
public class MavlinkFrameReader {
    //데이터 처리에 사용할 데이터를 최소단위로 변환하여 생성하여 보낸다.
    //이 때 변환한 데이터들은 전부 in에 저장된다
    private final TransactionalInputStream in;

    /**
     * Creates a mavlink frame reader for the specified input stream.
     *
     * @param in The input stream to read mavlink frames from.
     */
    public MavlinkFrameReader(InputStream in) {
        this.in = new TransactionalInputStream(in, 280);
    }

    /**
     * Reads the next frame in the stream. If this a call to this method returns {@code true},
     * then the bytes of the read frame can be retrieved by calling {@link #frame()}.
     *
     * @return {@code true} if the next frame was read, or {@code false} if the stream
     * has ended.
     * @throws IOException if an IO error occurs.
     */
    public boolean next() throws IOException {  //오류로 인한 종료를 막기 위해 로그로 오류를 던진다
        int versionMarker;
        int payloadLength;
        int incompatibleFlags;

        in.commit();
        while ((versionMarker = in.read()) != -1) {
            if ((payloadLength = in.read()) == -1) {
                return false;
            }
            switch (versionMarker) {
                case MavlinkPacket.MAGIC_V1:
                    return in.advance(6 + payloadLength);
                case MavlinkPacket.MAGIC_V2:
                    return (incompatibleFlags = in.read()) != -1
                            && in.advance(9 + payloadLength + (incompatibleFlags & 1) * 13);
                default:
                    drop();
            }
        }
        return false;
    }

    /**
     * Returns the frame read by the previous call to {@link #next()}.
     *
     * @return The read frame's bytes, or an empty byte array if no frame was read yet.
     */
    public byte[] frame() {
        return in.getBuffer();
    }   //프레임에 대한 바이트를 얻는다

    /**
     * Drops the last frame, returning its bytes to the stream, skipping its first byte.
     *
     * @throws IOException if an IO error occurs.
     */
    public void drop() throws IOException { //초기상태로 돌아간다
        in.rollback();
        in.skip(1);
        in.commit();
    }
}
