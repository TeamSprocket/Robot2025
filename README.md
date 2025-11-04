Hi guys, this is your first guided FRC learning module. Read the full text carefully and concisely.

For this first module, you'll be guided through how to complete basic FRC functions, which includes:

-creating and defining a motor
-basic running of a motor
-basic methods to get and set values
-basic connections with the robot container

Eric should have run through everything with you guys already, but let's start with the format of the code.

Our code is structure in the format of folders and files, like many other applications. The purpose of the code is to pass through controls from the controller to the robot, so it is our job to help interpret the values and send them to be displayed. That's all code is, it's interpretation of a task. So to do this, we start from the basis of our code. Our code starts in Main.java, where it activates and becomes "sentient", in a sense. Every piece of code starts here, which is partly the reason why we don't touch it. Main.java has one purpose: activate Robot.java, which is where the magic really happens. This is where all the parts get initialized, the power begins to flow, in a more hypothetical sense. The robot gets initialized here, and the code gets "built", meaning that every single class is created, every object called, and every method defined. Search up the definition of building code if you're unsure.

From then on, we go into the RobotContainer. Here, this is the "hub" of all the operations: defining the joycons and calling on which subsystems to run a method. 

Every time a RobotContiner method gets called, you go to a subsystem. Here, we only have one subsystem, but in a typical format, we have many. Each subsystem has its own variables and own information that it stows, whether the information is static or dynamic. To convey that information, or to make the subsystem do something, it uses its methods to transfer information across. Think of them as telephone lines, methods allow you to transfer information between subsystems and also tell them to do stuff.

A couple of other files you should be aware of:
Constants.java is full of constants, for when we have values we want to test or values that are just too specific to be placed to where we can't really remember them. It is primarily for access, so that we can access values quickly when we want to look up or change something.
The Util folder is full of useful methods and things like those, mainly for lengthy calculations that would benefit more from being in a separate file. 
Your vendordeps are your libraries; they provide the basis of what you do, and a lot of the pre-determined classes and methods come from them. 

From here on out, navigate to Subsystem.java and begin the module from there.