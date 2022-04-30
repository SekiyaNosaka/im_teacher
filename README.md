# im_teacher

## 概要
ROSの機能であるInteractive_Marker(以下 IM)を利用した、ナックル円形状の姿勢推定用教師データ作成ツール

## 実装詳細
- src/im_teacher_node.cpp: EnsensoのTFフレームを視点としたIMを発現及び操作できるコード
- scripts/dataset_collect.py: ナックル円形状の点群と姿勢をキー入力保存していくコード
- scripts/im_sub.py: 只のテストコード

## 作成されるデータ
作成されるデータは主に2つ1セット
以下に、2つの情報の詳細を記す。

- 教師データ
    - 入力: 点群
        - shape: (x,3)
          ※ xは任意の点群数
    - 出力: 円形状の姿勢
        - shape: (3,)

## 実行方法
* 2021/09/16時点

```
$ roslaunch tuduki_ur5_arm_config demo.launch

$ roslaunch bin_picking bin_picking.launch

$ workon detectron2_ros
$ roslaunch detectron2_ros 2class_knuckle_detectron2.launch

# --- 以上が，都筑のビンピッキングを走らせるローンチ
# --- 以下が，im_teacherの実行ノード

$ rosrun im_teacher im_teacher_node

$ python dataset_collect.py
# -> Rviz上でIMを任意に動かしてsubで値を受け取らせてからキー入力
```
