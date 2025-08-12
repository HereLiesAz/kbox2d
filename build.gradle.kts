
import org.jetbrains.kotlin.gradle.tasks.KotlinCompile

plugins {
    kotlin("jvm") version "1.9.21" apply false
    application
    id("org.openjfx.javafxplugin") version "0.0.13" apply false
}

allprojects {
    group = "com.hereliesaz.kbox2d"
    version = "1.0.0-SNAPSHOT"


    repositories {
        mavenCentral()
    }
}

subprojects {
    apply(plugin = "org.jetbrains.kotlin.jvm")

    tasks.withType<KotlinCompile> {
        kotlinOptions.jvmTarget = "11"
    }
}

