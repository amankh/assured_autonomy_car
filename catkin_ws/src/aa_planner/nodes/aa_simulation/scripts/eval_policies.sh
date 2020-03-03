#!/bin/bash
if [ $# -lt 2 ] || [ ! -d "$2" ]; then
   echo "Usage: $0 num_paths exp_dir [exp_dir ...]"
   exit 1
fi
if [ "$1" -ge 1 ]; then :
else
   echo "Usage: $0 num_paths exp_dir [exp_dir ...]"
   echo "   num_paths must be a number >= 1"
   exit 1
fi
NUM_PATHS="$1"
shift
if [ $# -ge 2 ]; then
   realpath -z "$@"| xargs -0 -n 1 -P "`nproc`" -- "$0" "${NUM_PATHS}"
else
   # This may need to be updated once we improve what is stored in the variant file
   if grep -q '"training script": ".*train_waypoint_controller.py"' "$1"/variant.json; then
      ENV=waypoint
   elif fgrep -q '"trajectory": "Straight",' "$1"/variant.json; then
      ENV=straight
   elif fgrep -q '"trajectory": "Circle",' "$1"/variant.json; then
      ENV=circle
   else
      echo "$0: Cannot figure out the environment type for $1!"
      exit 2
   fi

   D="`realpath "$1"`"
   cd "`dirname $0`"
   cd ../..
   # Make sure the eval_policy is time stamped at the time before params.pkl are first read
   TMP=`mktemp /tmp/eval_policies.XXXX`
   sleep 1
   if python3 aa_simulation/scripts/eval_policy.py "$D"/params.pkl --variant "$D"/variant.json --env "$ENV" --no-plots --seed 2 --num_paths "${NUM_PATHS}" > "$D"/eval_policy.txt; then
      touch -r "$TMP" "$D"/eval_policy.txt
      rm -f "$TMP"
      sed -e 's/^/'"`basename $D`"': /' <  "$D"/eval_policy.txt
   else
      rm -f "$D"/eval_policy.txt
   fi
fi
