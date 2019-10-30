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
https://drive.google.com/open?id=1lx8I6YBxZfL1GPhiJA9_YG0cYt4AD0Wy

#### Text detection
```
Path : semantic_lables_sys/catkin_ws/src/text_detection/textsnake/weights
```
https://drive.google.com/open?id=1lXbArdtTTvwlO_NhkgohHtasmfBMCNmw

### How to run the Brandname segmentation node

```
roslaunch semantic_system main.launch
```

### Object list

`
catkin_ws/src/text_msgs/config/commodity_list.txt
`
