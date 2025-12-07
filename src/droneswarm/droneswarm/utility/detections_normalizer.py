
from our_custom_interfaces.msg import ObjectData
import droneswarm.utility.settings as setting

def normalize_detection(err_x: float, err_y: float, bbox_w: float, bbox_h: float) -> tuple[float, float, float]:
    """ Returns a normalized detection tuple based on image size and BBOX_EDGE_LENGTH """

    # normalize to [-1, 1]
    n_err_x = err_x / (const.IMAGE_WIDTH / 2.0)
    n_err_y = err_y / (const.IMAGE_HEIGHT / 2.0)

    avg_edge_size = (bbox_w + bbox_h) / 2.0

    # normalise bounding box to a desired average edge length. If n_edge_dist is > 0 then then UAV is to far. If < 0 then its too close.  
    n_edge_dist = (setting.BBOX_EDGE_LENGTH - avg_edge_size) / setting.BBOX_EDGE_LENGTH
    # n_edge_dist = max(-2.0, min(2.0, n_edge_dist))

    return (n_err_x, n_err_y, n_edge_dist)