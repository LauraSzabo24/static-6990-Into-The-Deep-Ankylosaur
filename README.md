This is the 2024-2025 season's code for team 6990 Static Void's second robot "Ankylosaurus" or "Ankylo".
ALL OF THIS CODE IS STILL IN PROGRESS WORK

Credits:
All code within this repository is from either the 2024-2025 FTC Roadrunner Quickstart, 
EasyOpenCV library or is the work of Laura Szabo from #6990 Static Void

Guide/Breakdown:
All important code files are found under TeamCode/src/main/java. Here there are 4 folders:

    -Autonomous: Items under the Meet1 folder are outdated programs written for the autonomous portion. The Red4Specimen is the current 
    code I'm using for our autonomous portion. The file titled Mailbox is a class I use to transfer data from the autonomous portion to 
    code for the teleOp portion. Under the detectors folder are all of the programs I wrote for camera vision. DetectorRunner is just that,
    a runner program. The more interesting parts are in the OrientationDetector file which is the pipeline I made to detect the 
    orientation of the game element blocks. Finaly, pointVector is just a helper class I wrote to better store the data.
    
    -TeleOp: There's only one file in here and that contains all the teleOp or rather driver control period's code. 
    
    -org/firstinspires/ftc/teamcode: These contain some of the files from the Roadrunner Quickstart. However, the majority are
    modified especially the ones found under the drive folder. Most of these modifications were done in order to implement automated
    teleOp/driver control period controls and the pinpoint odometry controler. Some of the files here are also in Kotlin.
    
    -RelicsFromCenterStageGame/TeleOp - These are from a previous season that I keep to use as reference. They can be ignored

Thank you for visiting this repository!! I hope the way I have the files isn't too confusing or messy. All of the important files
should have comments to help sort out the chaos. Have a nice day!! 
