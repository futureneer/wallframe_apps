#! /bin/sh

if [ $# -ne 3 ]
then
  echo "Usage: `basename $0` image_dir image_glob WxH"
  echo "Example glob pattern: *@(.jpg|.jpeg)"
  exit 1
fi

imagedir=$1
imageglob=$2
thumbsize=$3
thumbdir=${imagedir}/thumbs

mkdir -p ${thumbdir}

shopt -s extglob

for file in ${imagedir}/${imageglob}
do
  echo "Creating thumb for $file"

  thumb=${thumbdir}/$(basename "${file%.*}").png
  convert "${file}" -resize ${thumbsize} "$thumb"
done

