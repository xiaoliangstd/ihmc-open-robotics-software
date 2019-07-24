plugins {
   id("us.ihmc.ihmc-build") version "0.19.5"
   id("us.ihmc.ihmc-ci") version "4.26"
   id("us.ihmc.ihmc-cd") version "1.4"
   id("us.ihmc.scs") version "0.4"
   id("us.ihmc.log-tools") version "0.3.1"
   application
}

ihmc {
   loadProductProperties("../product.properties")

   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   compile("com.martiansoftware:jsap:2.1")
   compile("org.yaml:snakeyaml:1.17") //1.11
   compile("org.ejml:core:0.30")
   compile("org.ejml:simple:0.30")
   compile("org.ejml:dense64:0.30")
   compile("com.esotericsoftware.minlog:minlog:1.2")
   compile("org.apache.commons:commons-lang3:3.8.1")
   compile("us.ihmc.thirdparty.jinput:jinput:190214")
   compile("org.ros.rosjava_bootstrap:message_generation:0.2.1")
   compile("org.ros.rosjava_messages:std_msgs:0.5.9")

   compile("us.ihmc:euclid:0.12.1")
   compile("us.ihmc:ihmc-yovariables:0.3.11")
   compile("us.ihmc:ihmc-realtime:1.2.6")
   compile("us.ihmc:IHMCRosControl:0.5.0") {
      setChanging(true)
   }
   compile("us.ihmc:ihmc-commons:0.26.6")
   compile("us.ihmc:ihmc-jmonkey-engine-toolkit:0.12.8")
   compile("us.ihmc:simulation-construction-set:0.12.15")
   compile("us.ihmc:ihmc-graphics-description:0.12.12")
   compile("us.ihmc:ihmc-robot-description:0.12.7")
   compile("us.ihmc:ihmc-communication:source")
   compile("us.ihmc:ihmc-humanoid-robotics:source")
   compile("us.ihmc:ihmc-system-identification:source")
   compile("us.ihmc:ihmc-state-estimation:source")
   compile("us.ihmc:ihmc-common-walking-control-modules:source")
   compile("us.ihmc:ihmc-avatar-interfaces:source")
   compile("us.ihmc:ihmc-ros-tools:source")
   compile("us.ihmc:ihmc-robot-data-logger:source")
   compile("us.ihmc:ihmc-model-file-loader:source")
   compile("us.ihmc:ihmc-sensor-processing:source")
   compile("us.ihmc:ihmc-perception:source")
   compile("us.ihmc:ihmc-whole-body-controller:source")
   compile("us.ihmc:ihmc-java-toolkit:source")
   compile("us.ihmc:ihmc-robotics-toolkit:source")
   compile("us.ihmc:ihmc-robot-models:source")
   compile("us.ihmc:ihmc-robot-data-visualizer:source")
   compile("us.ihmc:ihmc-simulation-toolkit:source")
   compile("us.ihmc:ihmc-footstep-planning-visualizers:source")
   compile("us.ihmc:ihmc-avatar-interfaces-behavior-fx-ui:source")
}

testDependencies {
   compile("us.ihmc:ihmc-commons-testing:0.26.6")
   compile("us.ihmc:ihmc-robotics-toolkit-test:source")
   compile("us.ihmc:ihmc-avatar-interfaces-test:source")
}

tasks.getByPath("installDist").dependsOn("compositeJar")

application.mainClassName = ""

val joystickApplication by tasks.creating(CreateStartScripts::class.java) {
   mainClassName = "us.ihmc.valkyrie.joystick.ValkyrieJoystickBasedSteppingApplication"
   applicationName = "IHMCValkyrieJoystickApplication"
   outputDir = File(project.buildDir, "scripts")
   classpath = tasks.getByName<Jar>("jar").outputs.files + project.configurations.runtime
}
val networkProcessorApplication by tasks.creating(CreateStartScripts::class.java) {
   mainClassName = "us.ihmc.valkyrie.ValkyrieROSNetworkProcessor"
   applicationName = "valkyrie-network-processor"
   outputDir = File(project.buildDir, "scripts")
   classpath = tasks.getByName<Jar>("jar").outputs.files + project.configurations.runtime
}
application.applicationDistribution.into("bin") {
   from(networkProcessorApplication)
   from(joystickApplication)
}

tasks.create("deployOCUApplications") {
   dependsOn("installDist")

   doLast {
      val appFolder = File(System.getProperty("user.home"), "ihmc_apps/valkyrie")
      appFolder.delete()
      appFolder.mkdirs()
      copy {
         from("build/install/valkyrie")
         into(appFolder)
      }
      println("-------------------------------------------------------------------------")
      println("------- Deployed files to: " + appFolder.path + " -------")
      println("-------------------------------------------------------------------------")
   }
}

tasks.create("deployLocal") {
   dependsOn("installDist")

   doLast {
      val libFolder = File(System.getProperty("user.home"), "valkyrie/lib")
      libFolder.delete()
      libFolder.mkdirs()

      copy {
         from("build/install/valkyrie/lib")
         into(libFolder)
      }

      copy {
         from("build/libs/valkyrie-$version.jar")
         into(File(System.getProperty("user.home"), "valkyrie"))
      }

      val configurationDir = File(System.getProperty("user.home"), ".ihmc/Configurations")
      configurationDir.delete()
      configurationDir.mkdirs()

      copy {
         from("saved-configurations/defaultREAModuleConfiguration.txt")
         into(configurationDir)
      }
   }
}

val directory = "/home/val/valkyrie"

tasks.create("deploy") {
   dependsOn("installDist")

   doLast {
      remote.session("link02", "val") // control
      {
         exec("mkdir -p $directory")

         exec("rm -rf $directory/lib")
         put("build/install/valkyrie/lib", "$directory/lib")
         exec("ls -halp $directory/lib")

         put("build/libs/valkyrie-$version.jar", "$directory/ValkyrieController.jar")
         put("launchScripts", directory)
         exec("ls -halp $directory")
      }

      deployNetworkProcessor()
   }
}

tasks.create("deployNetworkProcessor") {
   dependsOn("installDist")

   doLast {
      deployNetworkProcessor()
   }
}

fun deployNetworkProcessor()
{
   remote.session("zelda02", "val") // perception
   {
      exec("mkdir -p $directory")

      exec("rm -rf $directory/bin")
      exec("rm -rf $directory/lib")

      put("build/install/valkyrie/bin", "$directory/bin")
      exec("chmod +x $directory/bin/valkyrie-network-processor")
      put("build/install/valkyrie/lib", "$directory/lib")
      exec("ls -halp $directory/lib")

      put("build/libs/valkyrie-$version.jar", "$directory/ValkyrieController.jar")
      put("launchScripts", directory)
      exec("ls -halp $directory")

      exec("rm -rf /home/val/.ihmc/Configurations")
      exec("mkdir -p /home/val/.ihmc/Configurations")
      put("saved-configurations/defaultREAModuleConfiguration.txt", ".ihmc/Configurations")
      exec("ls -halp /home/val/.ihmc/Configurations")
   }
}