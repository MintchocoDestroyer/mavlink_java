package io.dronefleet.mavlink.annotations;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

//마브링크 병렬화 인터페이스
@Retention(RetentionPolicy.RUNTIME)
@Target({ElementType.TYPE})
public @interface MavlinkEnum {
}
