# Instructions for Assembly and Printing 

Generally speaking, the printing process should be quite ‘plug and play’ with minimal adaptations required. Obviously, if you use different circuit boards or BLDC motors, the dimensions will have to be adjusted. This is why we include both STL files ready for printing and the original Solidworks files.

Below are some of the specifications used for printing the existing version:

 - Printer: Ultimaker S5, staged in Cura
 - Material: Basic non-branded PLA, black. 0.4mm nozzle
 - Temperature: 205c for hot end, 60c for build plate
 - Layer Height: Between 0.1mm and 0.2mm layer height. We used 0.15mm.
 - Infill: Nothing more than 20% is necessary, we used 18% on the ‘gyroid’ setting.
 - Support and Adhesion: Support structures for overhangs, 8mm skirt for adhesion.

All parts should be printed exactly at their built size (100% scale), as certain dimensions have already been adjusted to account of tolerances and minimal shrinkage while printing.

The **one** exception to this is the ‘Knob’ part. The inner diameter is intentionally a hair too small, and thus one will need to scale this knob up for a perfect ‘force fit’ with your BLDC motor. We found around 100.5% or 101% of the original size worked. Of course, your mileage may vary based on your material choice, print speed, build environment temperature, and other factors. Adjust and test as suits your use case!

All parts are to be printed one time, with the exception of the pins (4 instances).

## Assembly Steps - Knob Unit 

 1. Take the part FK_AssmBasePlate as the first component, and take 4 round magnets to insert into each hole. Use a 5th magnet on the underside of each to orient all the magnets with the same polarity.

 2. Take the 4 copies of FK_AssmPeg and, using superglue, adhere above the holes with the magnets. One could glue the magnets themselves into the holes if you desire.
&emsp;a. One could also use one of the next parts in the stack (FK_MT6701Holder or FK_BLDCMount) the keep these pins aligned with proper tilt while the glue dries.

 3. Take the prepared MT6701 magnetic sensor and wires and insert it into the FK_MT6701Holder housing, feeding through the wires. Pay attention to orientation, as the cutouts allow space for underside resistors to help the board lay flat.
&emsp;a. After this step, the wires can be bundled.

 5. Attach the parts from step 4 to the cured AssmBasePlate, using small amounts of super glue on the larger diameter of each pin to secure the housing. Press down firmly to guarantee parallelism.

 6. Take the prepared BLDC motor and wires from step 14 of the electronics assembly guide and add this to FK_BLDCMount (and then both to the built of stack of parts from steps 1-5). Again, utilize small amounts of glue, this time hot glue, in the mounting holes of the housing, to secure items together.
&emsp;a. **Make sure the all wires are facing the same orientation and match with the cutouts designed into the parts.**

 7. Now, all of the wire bundles can themselves be bundled together using heat shrink tubing or whatever fastening mechanisms you prefer.

## Assembly Steps - Table Base 

 1. Take FK_TableBase and fill each of the 4 cylindrical holes with 12 washers (30mm diameter, 1mm thickness).

 2. To avoid noise and movement, these washers can be centrally filled with hot glue.

 3. After the washers are inserted, the 2 FK_ElectronicsLid parts can be used to cover the washers.
&emsp;a. While these are pretty secure by themselves, we recommend super or hot glue to fasten them down permanently.

 4. After these components are dried and adhered, flip the parts over to reveal the magnet holes on the bottom.

 5. You can choose how many magnets to place in the device t ovary the attractive force between the knob assembly and this base for the table top (we personally used 2).

 6. Place the knob assembly from the prior section into place on the FK_TableBase, and THEN insert the magnets into the deep holes, as this will properly align their polarity.

 7. Follow the magnets with generous hot glue to seal the holes. Let dry.

 8. Flip right-side up and test the removal force of the knob assembly from the base. is it not strong enough? Add a 3rd (and 4th) magnet to the base part.

*This should mark the full assembly of all parts!*

> Written with [StackEdit](https://stackedit.io/).
