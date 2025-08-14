pluginManagement {
    repositories {
        gradlePluginPortal()
        mavenCentral()
        maven {
            url = uri("https://plugins.gradle.org/m2/")
        }
    }
}

pluginManagement {
    repositories {
        gradlePluginPortal()
        mavenCentral()
        google()
    }
}

rootProject.name = "kbox2d"

include(
    ":kbox2d-library",
    ":kbox2d-serialization-kt",
    ":kbox2d-testbed-javafx",
    ":kbox2d-testbed-javafx-kt",
    ":jbox2d-testbed-jogl"
)
