rootProject.name = "season-2025"
enableFeaturePreview("TYPESAFE_PROJECT_ACCESSORS")
pluginManagement {
    plugins {
        // Versions at https://plugins.gradle.org/plugin/edu.wpi.first.GradleRIO
        id("edu.wpi.first.GradleRIO") version "2025.1.1"
        // Versions at https://kotlinlang.org/docs/releases.html#release-details
        kotlin("jvm") version "2.1.0"
    }

    repositories {
        mavenLocal()
        mavenCentral()
        gradlePluginPortal()
        maven {
            name = "wpilib"
            url = uri("~/wpilib/2025/maven")
        }
        if("windows" in System.getProperty("os.name").lowercase()) {
            maven {
                name = "wpilib-alt"
                url = uri("/Users/Public/wpilib/2025/maven")
            }
        }
    }
}

dependencyResolutionManagement {
    repositoriesMode.set(RepositoriesMode.PREFER_PROJECT)
    repositories {
        google()
        mavenCentral()
        mavenLocal()
        maven {
            name = "wpilib"
            url = uri("~/wpilib/2025/maven")
        }
        if("windows" in System.getProperty("os.name").lowercase()) {
            maven {
                name = "wpilib-alt"
                url = uri("/Users/Public/wpilib/2025/maven")
            }
        }
        maven("https://frcmaven.wpi.edu/artifactory/release/")
        maven("https://dev.studica.com/maven/release/2025/")
        maven("https://maven.ctr-electronics.com/release/")
        maven("https://maven.revrobotics.com/")
        maven("https://maven.reduxrobotics.com/")
        maven("https://docs.home.thethriftybot.com")
        maven("https://repo1.maven.org/maven2")
        maven("https://shenzhen-robotics-alliance.github.io/maple-sim/vendordep/repos/releases")
        maven("https://broncbotz3481.github.io/YAGSL-Lib/yagsl/repos")
    }
}

include(":robot")
