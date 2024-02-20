package io.dronefleet.mavlink.annotations;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

//마브링크 메세지 정보 인터페이스
@Retention(RetentionPolicy.RUNTIME)
@Target({ElementType.TYPE})
public @interface MavlinkMessageInfo {
    int id();

    int crc();

    String description() default "No description provided";

    boolean workInProgress() default false;
}
