steps to calculate object pose

goal:
    pose2d of object - always assume object is on the ground

inputs:
    id of object
    horizontal and vertical angle to center of object from center of camera
    area of object in camera sensor (we probably don't need this)
    corners of the object in pixels



when calculating ray intersection with ground or walls add in half the width of the object since the object itself has size

calculation:
if not intersecting an edge
1) calculate pose3d
    use width, hieght, or both to determine how far the object is from the camera
    we get raw pixels but we can calculate pixels per degree and work from there
    since we know the width/height of the object we can easily get distance from degrees
    pixels we would need to plot in excel and get an equation
    i think i like ppd more. we can precalcuate for every camera
2) if pose3d is below the ground use the ray intersection to calculate pose2d of the object on the ground
3) else create pose2d at pose3d (i.e. move it to the ground)
    do we want to reject poses that are too high. a bouncing object could result in oscillating positions
    do we also reject poses that are too low under the ground
3) if pose2d is outside the perimeter use the ray intersection to calculate an adjusted pose2d of the object inside the perimeter
    the april tag poses do not take into account the cutouts at the source. since we are intaking at these points it would be good to move the object pose inside the actual field perimeter

if intersecting an edge that does not inhibit distance calculation do the above
    i.e. using vertical pixels but overlapping left edge
    we can calculate distance, but not angle. shouldn't be too much of an issue since if it is the only object on screen we will point the robot at it and will be able to get an acurate angle when it gets in full view.
    ideally we are targeting a ball that is the same both vertically and horizontally

if an intersecting edge does inhibit distance calculation
    we should still calculate something since we may want to target that piece
    if that piece is close and we also see a far away piece we would still want to target the closer one

if intersecting bottom edge
    set a position close to the front of the robot




calculate a single object to target:
I would want to contain this to the vision file

we determine a single target for other code to go to
target the closest to the center of the camera
if an object is close to the edge but closer, then choose that instead
we can adjust this with an equation

this assumes that the vision data is stable and maintains a good read of the piece we are targeting
if it does we could rapidly change targets
we will just have to test and tune this

I don't know how exactly to handle getting too close to an object
we need to keep targeting the correct object
if the camera can't see the intake this becomes so much harder since we will need to remember objects we no longer see. let's just not put a camera in this situation
if the camera can still see the intake, as we suck in the pose could have a tendency to move away as more of the piece is obscured
we could monitor the intake torque and "lock" the position when it goes up (do this in the intake command, not in the vision file)
we could also use the height to attempt to get an accurate ish pose and keep it from pusing away
if the camera is looking at a low enough angle the ground ray could be enough to counteract most of this




depending on how we move the the target it should be able to handle rolling targets
we will just have to test this
