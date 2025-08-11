import org.jetbrains.kotlin.gradle.tasks.KotlinCompile

plugins {
    `java-library`
    kotlin("jvm")
}

java {
    toolchain {
        languageVersion.set(JavaLanguageVersion.of(17))
    }
}

dependencies {
    testImplementation("junit:junit:4.13.1")
    testImplementation(kotlin("test"))
}

sourceSets {
    main {
        java {
            exclude("**/gwtemul/**")
        }
    }
}
