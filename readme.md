Pi-Tag Detector
===============

This is a fork of the Pi-Tag fiducial marker detector in the
[`cob_fiducials`](http://wiki.ros.org/cob_fiducials) [ROS](http://www.ros.org/)
module, modified to not require ROS. Other modifications include a separate
function to just get the marker pixel coordinates instead of the pose and the
addition of the [ellipse refinement step](https://doi.org/10.1007/s00138-008-0141-3)
mentioned in the [Pi-Tag paper](https://doi.org/10.1007/s00138-012-0469-6).

One example marker is included in the `markers` directory, copied from
`cob_fiducials`. Others can be generated with a
[Pi-Tag generator](https://github.com/raultron/PiTag-generator).

The included test program can be invoked with `./pitag-test modelfile imagefile`,
which will print a list of detected tags and the image pixel locations of their
dots.



Dependencies
-----------

 * [OpenCV](http://opencv.org/)
 * [TinyXML](http://www.grinninglizard.com/tinyxml/)
 * [CMake](https://cmake.org/)


License
-------

 * [LGPL v3+](https://www.gnu.org/licenses/lgpl-3.0.en.html) (per the `cob_fiducials` license)
 * The ellipse refinement step, copied from the
   [RUNE-Tag](http://www.dsi.unive.it/~bergamasco/runetag/) reference
   implementation, and the test program are licensed under the
   [MIT license](https://opensource.org/licenses/MIT)


Credits
-------

 * Original detector implementation: [`cobs_fiducials`](https://github.com/ipa320/cob_object_perception)
 * Ellipse refinement: [RUNE-Tag reference implementation](http://www.dsi.unive.it/~bergamasco/runetag/)
 * Pi-Tag paper: [Bergamasco, Filippo, Andrea Albarelli, and Andrea Torsello. "Pi-tag: a fast image-space marker design based on projective invariants." Machine vision and applications 24, no. 6 (2013): 1295-1310.](https://doi.org/10.1007/s00138-012-0469-6)
 * This standalone detector: [Matthew Petroff](https://mpetroff.net/)
