# baxter-cube-solver
This repo will hold all of the material related to my honors project, where I will try to enable Baxter, a two armed robot in the UML Robotics Lab, to solve a Rubik's cube. :)

Current Work
Baxter successfully moves his arm over the cube.  Next steps are to use computer vision to fine tune his position as he lowers his arm and turns his wrist to pick up the cube.
   - open the grippers
   - use square detection to find the cube
   - in a  loop to make sure that we stay on target: 
      - fix orientation
          * find diagonal vector 
          * use the angle of this vector to determine the offset from desired position
          * twist wrist until desired orientation is achieved
      - fix position
          * find desired centroid
          * find centroid of cube
          * calculate offset from desired centroid
          * move the hand into the new orientation
      - step down in z (move closer to the cube  
  - when z value is appropriate, close the grippers
  - move arms into a desired position for manipulating the cube
  
