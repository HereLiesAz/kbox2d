import org.gradle.api.tasks.testing.Test

plugins {
    `java-library`
}

sourceSets {
    main {
        java {
            exclude("**/gwtemul/**")
        }
    }
}

tasks.named<Test>("test") {
    enabled = false
}

java {
    toolchain {
        languageVersion.set(JavaLanguageVersion.of(17))

    }
}
