rootProject.name = "kbox2d"

include(
    "kbox2d-library",
pluginManagement {
    repositories {
        gradlePluginPortal()
        mavenCentral()
        maven {
            url = uri("https://plugins.gradle.org/m2/")
        }
    }
}

rootProject.name = "kbox2d"
include(":kbox2d-library")
include(":kbox2d-serialization-kt")
include(":kbox2d-testbed-javafx-kt")
include(":kbox2d-jni-broadphase-kt")
include(":kbox2d-testbed-jogl-kt")
include(
    "kbox2d-library",
    "jbox2d-testbed-jogl"

    "kbox2d-serialization",
    "kbox2d-testbed",
    "kbox2d-testbed-javafx",
    "kbox2d-jni-broadphase",
    "kbox2d-testbed-jogl"
)

