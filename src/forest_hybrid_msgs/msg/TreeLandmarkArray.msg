# Conjunto de árvores detectadas num único frame pela perceção LiDAR.
# O header fixa o stamp e o frame_id (base_link) de todas as deteções.
std_msgs/Header header
TreeLandmark[] trees
