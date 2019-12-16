# semantic_lables_sys

## Clone the repo

```
git clone --recursive https://github.com/AndySer37/semantic_labels_sys/
```

## Git Pull & submodules

```
git pull --recurse-submodules
```

### Download the models

#### Text recognize
```
Path : semantic_lables_sys/catkin_ws/src/text_recognize/moran_text_recog/weights
```
https://drive.google.com/file/d/1tv8osHoHazyJgErZSK6Jr4wLIGI-_fyS/view?usp=sharing

#### Text detection
```
Path : semantic_lables_sys/catkin_ws/src/text_detection/textsnake/weights
```
https://drive.google.com/file/d/1NMuvf8G3FpyqEnrqwOC3TBLfZ47gyOwJ/view?usp=sharing

### How to run the Brandname segmentation node

```
roslaunch semantic_system main.launch
```
Turn on the service

```
rosservice call /text_detection/predict_switch_server "data: true"
```

### Object list

`
catkin_ws/src/text_msgs/config/commodity_list.txt
`
