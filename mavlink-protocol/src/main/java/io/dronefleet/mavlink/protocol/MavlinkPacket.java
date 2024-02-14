package io.dronefleet.mavlink.protocol;

import io.dronefleet.mavlink.protocol.util.CrcX25;

import java.security.MessageDigest;
import java.security.NoSuchAlgorithmException;
import java.util.Arrays;

/**
 * <p>
 * Facilitates a Mavlink protocol packet. This class includes functionality for
 * serializing, deserializing, signing and CRC validation of Mavlink protocol packets.
 */
public class MavlinkPacket {
    //어느쪽을 사용할 것인지에 대한 마스킹 연산 준비
    static final int MAGIC_V1 = 0xFE;   //0b00000001
    static final int MAGIC_V2 = 0xFD;   //0b00000010
    private static final int INCOMPAT_FLAG_SIGNED = 0x01;
    //호환 되지 않는 패킷에 대해 플래그를 세워 마스킹

    /**
     * 신뢰성있는 마브링크2 패킷을 생성한다
     *
     * @param sequence    패킷을 만들기 위한 과정.
     * @param systemId    해당 패킷 발행자의 시스템 ID.
     * @param componentId 해당 패킷의 발신자의 구성 요소 ID.
     * @param messageId   해당 패킷의 메세지 ID.
     * @param crcExtra    해당 패킷의 메시지 ID에 해당하는 CRC 여분.
     * @param payload     해당 패킷의 페이로드.
     * @param linkId      해당 패킷 서명에 사용할 링크 ID.
     * @param timestamp   해당 패킷의 서명에 사용할 타임스탬프.
     * @param secretKey   해당 패킷의 서명을 생성하는 데 사용할 비밀 키.
     * @return 지정된 설정의 서명된 마브링크2 패킷.
     */
    public static MavlinkPacket createSignedMavlink2Packet(
            int sequence, int systemId, int componentId, int messageId,
            int crcExtra, byte[] payload, int linkId, long timestamp, byte[] secretKey) {
        byte[] rawBytes = new byte[25 + payload.length];
        ByteArray bytes = new ByteArray(rawBytes);
        bytes.putInt8(MAGIC_V2, 0);
        bytes.putInt8(payload.length, 1);
        bytes.putInt8(INCOMPAT_FLAG_SIGNED, 2);
        bytes.putInt8(0, 3);
        bytes.putInt8(sequence, 4);
        bytes.putInt8(systemId, 5);
        bytes.putInt8(componentId, 6);
        bytes.putInt24(messageId, 7);
        System.arraycopy(payload, 0, rawBytes, 10, payload.length);
        int crc = generateCrc(rawBytes, crcExtra);
        bytes.putInt16(crc, 10 + payload.length);
        byte[] signature = generateSignature(rawBytes, linkId, timestamp, secretKey);
        System.arraycopy(signature, 0, rawBytes, rawBytes.length - signature.length, signature.length);
        return new MavlinkPacket(MAGIC_V2, INCOMPAT_FLAG_SIGNED, 0, sequence, systemId,
                componentId, messageId, payload, crc, signature, rawBytes);
    }

    /**
     * 신뢰성이 없는 마브링크2 패킷을 생성한다.
     *
     * @param sequence    패킷을 만들기 위한 과정.
     * @param systemId    해당 패킷 발행자의 시스템 ID.
     * @param componentId 해당 패킷의 발신자의 구성 요소 ID.
     * @param messageId   해당 패킷의 메세지 ID.
     * @param crcExtra    해당 패킷의 메시지 ID에 해당하는 CRC 여분.
     * @param payload     해당 패킷의 페이로드.
     * @return 지정된 설정의 서명된 마브링크2 패킷.
     */
    public static MavlinkPacket createUnsignedMavlink2Packet(
            int sequence, int systemId, int componentId, int messageId,
            int crcExtra, byte[] payload) {
        byte[] rawBytes = new byte[12 + payload.length];
        ByteArray bytes = new ByteArray(rawBytes);
        bytes.putInt8(MAGIC_V2, 0);
        bytes.putInt8(payload.length, 1);
        bytes.putInt8(0, 2);
        bytes.putInt8(0, 3);
        bytes.putInt8(sequence, 4);
        bytes.putInt8(systemId, 5);
        bytes.putInt8(componentId, 6);
        bytes.putInt24(messageId, 7);
        System.arraycopy(payload, 0, rawBytes, 10, payload.length);
        int crc = generateCrc(rawBytes, crcExtra);
        bytes.putInt16(crc, 10 + payload.length);
        return new MavlinkPacket(MAGIC_V2, 0, 0, sequence, systemId, componentId, messageId,
                payload, crc, new byte[0], rawBytes);
    }

    /**
     * 마브링크1 패킷을 생성한다.
     *
     * @param sequence    패킷을 만들기 위한 과정.
     * @param systemId    해당 패킷 발행자의 시스템 ID.
     * @param componentId 해당 패킷의 발신자의 구성 요소 ID.
     * @param messageId   해당 패킷의 메세지 ID.
     * @param crcExtra    해당 패킷의 메시지 ID에 해당하는 CRC 여분.
     * @param payload     해당 패킷의 페이로드.
     * @return 지정된 설정의 서명된 마브링크2 패킷.
     */
    public static MavlinkPacket createMavlink1Packet(
            int sequence, int systemId, int componentId, int messageId,
            int crcExtra, byte[] payload) {
        byte[] rawBytes = new byte[8 + payload.length];
        ByteArray bytes = new ByteArray(rawBytes);
        bytes.putInt8(MAGIC_V1, 0);
        bytes.putInt8(payload.length, 1);
        bytes.putInt8(sequence, 2);
        bytes.putInt8(systemId, 3);
        bytes.putInt8(componentId, 4);
        bytes.putInt8(messageId, 5);
        System.arraycopy(payload, 0, rawBytes, 6, payload.length);
        int crc = generateCrc(rawBytes, crcExtra);
        bytes.putInt16(crc, 6 + payload.length);
        return new MavlinkPacket(MAGIC_V1, -1, -1, sequence, systemId, componentId, messageId,
                payload, crc, new byte[0], rawBytes);
    }

    /**
     * 마브링크 1 패킷을 파싱한다.
     *
     * @param rawBytes 마브링크 1 패킷, 패킷 프레임을 해석.
     * @return 마브링크 1 패킷을 다시 파싱한다.
     */
    public static MavlinkPacket fromV1Bytes(byte[] rawBytes) {
        ByteArray bytes = new ByteArray(rawBytes);
        int versionMarker = bytes.getInt8(0);
        int payloadLength = bytes.getInt8(1);
        int sequence = bytes.getInt8(2);
        int systemId = bytes.getInt8(3);
        int componentId = bytes.getInt8(4);
        int messageId = bytes.getInt8(5);
        byte[] payload = bytes.slice(6, payloadLength);
        int checksum = bytes.getInt16(6 + payloadLength);
        return new MavlinkPacket(versionMarker, -1, -1, sequence, systemId, componentId,
                messageId, payload, checksum, new byte[0], rawBytes);
    }

    /**
     * 마브링크 2 패킷을 파싱한다.
     *
     * @param rawBytes 마브링크 2 패킷, 패킷 프레임을 해석.
     * @return 마브링크 2 패킷을 다시 파싱한다.
     */
    public static MavlinkPacket fromV2Bytes(byte[] rawBytes) {
        ByteArray bytes = new ByteArray(rawBytes);
        int versionMarker = bytes.getInt8(0);
        int payloadLength = bytes.getInt8(1);
        int incompatibleFlags = bytes.getInt8(2);
        int compatibleFlags = bytes.getInt8(3);
        int sequence = bytes.getInt8(4);
        int systemId = bytes.getInt8(5);
        int componentId = bytes.getInt8(6);
        int messageId = bytes.getInt24(7);
        byte[] payload = bytes.slice(10, payloadLength);
        int checksum = bytes.getInt16(10 + payloadLength);
        byte[] signature;
        if ((incompatibleFlags & INCOMPAT_FLAG_SIGNED) != 0) {
            signature = bytes.slice(12 + payloadLength, 13);
        } else {
            signature = new byte[0];
        }
        return new MavlinkPacket(versionMarker, incompatibleFlags, compatibleFlags, sequence,
                systemId, componentId, messageId, payload, checksum, signature, rawBytes);
    }

    /**
     * CRC 생성.
     *
     * @param packetBytes CRC를 생성할 패킷 바이트. 이 메서드는
     *                    예상 위치까지의 바이트만 소비
     *                    CRC의 경우, 더 긴 프레임을 통과시키는 것이 안전
     *                    프레임의 CRC 위치보다. 패킷 바이트는
     *                    버전 마커(또는 STX)가 포함될 것으로 예상됨
     * @param crcExtra    해당 패킷의 메시지의 CRC 추가 항목.
     * @return CRC 체크섬 생성.
     */
    public static int generateCrc(byte[] packetBytes, int crcExtra) {
        if (packetBytes.length < 3) {
            return -1;
        }
        int payloadLength = packetBytes[1] & 0xFF;
        int packetLengthWithoutCrc;
        switch (packetBytes[0] & 0xFF) {
            case MAGIC_V1:
                packetLengthWithoutCrc = 6;
                break;
            case MAGIC_V2:
                packetLengthWithoutCrc = 10;
                break;
            default:
                throw new IllegalStateException("not a mavlink packet");
        }
        packetLengthWithoutCrc += payloadLength;
        CrcX25 crc = new CrcX25();
        crc.accumulate(packetBytes, 1, packetLengthWithoutCrc);
        crc.accumulate(crcExtra);
        return crc.get();
    }

    /**
     * 마브링크 2의 서명 생성.
     *
     * @param packetBytes 서명을 생성할 패킷의 바이트 수.
     * @param linkId      서명의 링크 ID.
     * @param timestamp   서명의 타임스탬프.
     * @param secretKey   서명의 해시를 생성하는 데 사용할 비밀 키.
     * @return 서명 생성.
     */
    public static byte[] generateSignature(
            byte[] packetBytes, int linkId, long timestamp, byte[] secretKey) {
        if (packetBytes.length < 3
                || (packetBytes[0] & MavlinkPacket.MAGIC_V2) != MavlinkPacket.MAGIC_V2
                || (packetBytes[2] & MavlinkPacket.INCOMPAT_FLAG_SIGNED) == 0) {
            return new byte[0];
        }
        int payloadLength = packetBytes[1] & 0xFF;
        int packetLengthWithCrc = 12 + payloadLength;
        if (packetBytes.length < packetLengthWithCrc) {
            throw new IllegalArgumentException("specified packet seems to be incomplete");
        }
        byte[] signature = new byte[13];
        ByteArray bytes = new ByteArray(signature);
        bytes.putInt8(linkId, 0);
        bytes.putInt48(timestamp, 1);
        try {
            MessageDigest digest = MessageDigest.getInstance("SHA-256");
            digest.update(secretKey);
            digest.update(packetBytes, 0, packetLengthWithCrc);
            digest.update(signature, 0, 7);
            byte[] hash = digest.digest();
            System.arraycopy(hash, 0, signature, 7, 6);
            return signature;
        } catch (NoSuchAlgorithmException e) {
            throw new IllegalStateException("JVM does not have an implementation of SHA-256 available.");
        }
    }

    private final int versionMarker;
    private final int incompatibleFlags;
    private final int compatibleFlags;
    private final int sequence;
    private final int systemId;
    private final int componentId;
    private final int messageId;
    private final byte[] payload;
    private final int checksum;
    private final byte[] signature;
    private final byte[] rawBytes;

    private MavlinkPacket(
            int versionMarker,
            int incompatibleFlags,
            int compatibleFlags,
            int sequence,
            int systemId,
            int componentId,
            int messageId,
            byte[] payload,
            int checksum,
            byte[] signature,
            byte[] rawBytes) {
        this.versionMarker = versionMarker;
        this.incompatibleFlags = incompatibleFlags;
        this.compatibleFlags = compatibleFlags;
        this.sequence = sequence;
        this.systemId = systemId;
        this.componentId = componentId;
        this.messageId = messageId;
        this.payload = payload;
        this.checksum = checksum;
        this.signature = signature;
        this.rawBytes = rawBytes;
    }

    /**
     * 해당 패킷의 버전 마커(STX)를 반환. 결과는 버전 1 패킷의 경우 0xFE,
     * 버전 2의 경우 0xFD.
     */
    public int getVersionMarker() {
        return versionMarker;
    }

    /**
     * 이 패킷의 비호환성 플래그를 반환합니다. 비호환성 플래그는 다음을 나타내는 데 사용됩니다
     * MAVLink 라이브러리가 패킷을 처리하기 위해 지원해야 하는 기능입니다.
     * 여기에는 패킷 형식/순서에 영향을 미치는 모든 기능이 포함됩니다. 만약 에 플래그가 있다면
     * 이 비트 마스크는 이해할 수 없으며 패킷을 삭제해야 합니다.
     */
    public int getIncompatibleFlags() {
        return incompatibleFlags;
    }

    /**
     * 이 패킷의 호환성 플래그를 반환합니다. 호환성 플래그는 기능을 나타내는 데 사용됩니다
     * 라이브러리가 패킷을 처리하는 것을 방해하지 않습니다(기능이 이해되지 않더라도).
     * 여기에는 예를 들어 패킷이 다음과 같이 처리되어야 함을 나타내는 플래그가 포함될 수 있습니다
     * "high priority"(이러한 메시지는 모든 MAVLink 구현에서 처리할 수 있습니다.)
     * 패킷 형식 및 구조는 영향을 받지 않습니다).
     */
    public int getCompatibleFlags() {
        return compatibleFlags;
    }

    /**
     * 이 패킷의 시퀀스 번호를 반환합니다. 시퀀스 번호는 0에서 255 사이의 값입니다,
     * 연속적인 패킷들 사이에서 증가합니다. 시퀀스 번호는 한 번 0으로 재설정됩니다
     * 사용 가능한 8비트를 오버플로우했습니다.
     */
    public int getSequence() {
        return sequence;
    }

    /**
     * 이 패킷의 발신자의 시스템 ID를 반환합니다.
     */
    public int getSystemId() {
        return systemId;
    }

    /**
     * 이 패킷의 발신자의 구성 요소 ID를 반환합니다.
     */
    public int getComponentId() {
        return componentId;
    }

    /**
     * 이 패킷의 메시지 ID를 반환합니다.
     */
    public int getMessageId() {
        return messageId;
    }

    /**
     * 이 패킷의 페이로드를 반환합니다. 페이로드 내용은 관찰을 통해 해결할 수 있습니다.
     * 이 패킷의 {@link #getMessageId() message id}입니다
     */
    public byte[] getPayload() {
        return payload;
    }

    /**
     * 이 패킷의 CRC 체크섬을 반환합니다.
     */
    public int getChecksum() {
        return checksum;
    }

    /**
     * 이 패킷의 서명을 반환합니다.
     *
     * @return 이 패킷의 서명 또는 서명이 없는 경우 빈 바이트 배열
     * 이 패킷에 포함되어 있습니다.
     */
    public byte[] getSignature() {
        return signature;
    }

    /**
     * 전체 패킷 바이트가 포함된 바이트 배열을 반환합니다
     * 송신 또는 수신.
     */
    public byte[] getRawBytes() {
        return rawBytes;
    }

    /**
     * 버전 2 패킷인지 확인합니다.
     *
     * @return 마브링크 2의 패킷이라면 {@code true} , 아니면 {@code false}.
     */
    public boolean isMavlink2() {
        return versionMarker == MAGIC_V2;
    }

    /**
     * 이 패킷의 CRC 체크섬을 확인합니다.
     *
     * @param crcExtra 이 패킷의 메시지 ID에 해당하는 여분의 CRC
     * @return 이 패킷의 CRC 체크섬이 유효성 검사를 통과한 경우 {@code true}, 아니면 {@code false}
     */
    public boolean validateCrc(int crcExtra) {
        return generateCrc(rawBytes, crcExtra) == checksum;
    }

    /**
     * 이 패킷이 서명되었는지 확인합니다.
     *
     * @return 이 패킷의 비호환성 플래그가 서명되었음을 나타내는 경우 {@code true}, 아니면 {@code false}
     */
    public boolean isSigned() {
        return ((incompatibleFlags & INCOMPAT_FLAG_SIGNED) != 0);
    }

    /**
     * 이 패킷의 서명을 확인합니다.
     *
     * @param secretKey 서명을 확인할 때 사용할 비밀 키.
     * @return 서명이 지정된 매개 변수에 따라 유효한 경우 {@code true}, 아니면 {@code false}
     */
    public boolean validateSignature(byte[] secretKey) {
        return isSigned() &&
                Arrays.equals(
                        signature,
                        generateSignature(
                                this.rawBytes,
                                getSignatureLinkId(),
                                getSignatureTimestamp(),
                                secretKey));
    }

    /**
     * 이 패킷의 서명의 링크 ID를 반환합니다.
     */
    public int getSignatureLinkId() {
        return isSigned() ? (signature[0] & 0xFF) : -1;
    }

    /**
     * 이 패킷의 서명 타임스탬프를 반환합니다.
     */
    public long getSignatureTimestamp() {
        ByteArray bytes = new ByteArray(signature);
        return isSigned() ? bytes.getInt48(1) : -1;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;

        MavlinkPacket that = (MavlinkPacket) o;

        if (versionMarker != that.versionMarker) return false;
        if (incompatibleFlags != that.incompatibleFlags) return false;
        if (compatibleFlags != that.compatibleFlags) return false;
        if (sequence != that.sequence) return false;
        if (systemId != that.systemId) return false;
        if (componentId != that.componentId) return false;
        if (messageId != that.messageId) return false;
        if (checksum != that.checksum) return false;
        if (!Arrays.equals(payload, that.payload)) return false;
        if (!Arrays.equals(signature, that.signature)) return false;
        return Arrays.equals(rawBytes, that.rawBytes);
    }

    @Override
    public int hashCode() {
        int result = versionMarker;
        result = 31 * result + incompatibleFlags;
        result = 31 * result + compatibleFlags;
        result = 31 * result + sequence;
        result = 31 * result + systemId;
        result = 31 * result + componentId;
        result = 31 * result + messageId;
        result = 31 * result + Arrays.hashCode(payload);
        result = 31 * result + checksum;
        result = 31 * result + Arrays.hashCode(signature);
        result = 31 * result + Arrays.hashCode(rawBytes);
        return result;
    }

    @Override
    public String toString() {
        return "MavlinkPacket{" +
                "versionMarker=" + versionMarker +
                ", incompatibleFlags=" + incompatibleFlags +
                ", compatibleFlags=" + compatibleFlags +
                ", sequence=" + sequence +
                ", systemId=" + systemId +
                ", componentId=" + componentId +
                ", messageId=" + messageId +
                ", payload=" + Arrays.toString(payload) +
                ", checksum=" + checksum +
                ", signature=" + Arrays.toString(signature) +
                '}';
    }
}
