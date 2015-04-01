#/bin/bash
DIR=../models/stress_$1
../../build/test ./stress-$1/robot.txt $DIR/model_raw.sdf 1.0
xmllint --format $DIR/model_raw.sdf > $DIR/model.sdf
