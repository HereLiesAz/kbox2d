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
