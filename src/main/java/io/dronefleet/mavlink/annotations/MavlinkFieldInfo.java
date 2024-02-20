package io.dronefleet.mavlink.annotations;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

//마브링크 필드 정보 인터페이스
@Retention(RetentionPolicy.RUNTIME)
@Target({ElementType.METHOD})
public @interface MavlinkFieldInfo {
    int position();

    int unitSize();

    boolean signed() default false;

    int arraySize() default 0;

    Class<?> enumType() default void.class;

    boolean extension() default false;

    String description() default "No description provided";
}
