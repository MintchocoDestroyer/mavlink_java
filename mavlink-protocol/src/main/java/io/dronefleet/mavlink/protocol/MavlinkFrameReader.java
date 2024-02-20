package io.dronefleet.mavlink.protocol;

import java.io.IOException;
import java.io.InputStream;

/**
 * <p>
 * 마브링크 프로토콜 프레임을 읽고오세요.
 * <p>
 * 프레임은 완전한 패킷처럼 보이는 바이트의 집합입니다. 프레임은 다음과 같이 시작합니다
 * 유효한 프로토콜 버전 마커(STX)이며, 예상되는 나머지 패킷 바이트를 포함합니다
 * 지정된 길이, 프로토콜 버전 및 비호환성 플래그(예를 들면 서명).
 * <p>
 *
 파싱된 프레임은 실제 테스트가 프레임뿐이라는 점에서 신뢰할 수 없습니다
 유효한 STX로 시작했습니다. 유효한 STX를 시작하는 바이트 수열은 다음과 같습니다
 이 독자에 의해 프레임으로 반환되었습니다. 따라서 이 클래스의 사용자는 이해해야 합니다
 CRC는 반환된 프레임을 확인하고, 유효성 검사에 실패할 경우 {@link #drop()}(으)로 호출합니다
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
     * 스트림에서 다음 프레임을 읽습니다. 이 메서드에 대한 호출이 {@code true}을(를) 반환하는 경우,
     * 그런 다음 {@link #frame ()}을(를) 호출하여 읽기 프레임의 바이트를 검색할 수 있습니다.
     *
     * @return {@code true}일 경우 다음 프레임을 읽고, {@code false}일 경우 스트림이 종료됩니다.
     * @throws IOException IO 에러가 났을 경우.
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
     * 이전 호출에서 읽은 프레임을 {@link #next()}(으)로 반환합니다.
     *
     * @return 읽기 프레임의 바이트 또는 아직 읽기 프레임이 없는 경우 빈 바이트 배열.
     */
    public byte[] frame() {
        return in.getBuffer();
    }   //프레임에 대한 바이트를 얻는다

    /**
     * 마지막 프레임을 드롭하고 해당 바이트를 스트림으로 되돌리며 첫 번째 바이트를 건너뜁니다.
     *
     * @throws IOException IO 에러가 났을 경우.
     */
    public void drop() throws IOException { //초기상태로 돌아간다
        in.rollback();
        in.skip(1);
        in.commit();
    }
}
