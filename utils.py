import numpy as np
import json


def normalize_angle(angle):
    """Normalize angle to [-pi, pi]."""
    return np.float32((angle + np.pi) % (2 * np.pi) - np.pi)

def load_json_feature(filename, interp_dist=0.1):
    with open(filename, 'r') as file:
        data = json.load(file)
        try:
            nfm_origin = data['Frames']['0']["m_nfmOrigin"]
        except:
            nfm_origin = [0, 0] 
        try:       
            m_pathOrigin = data['Frames']['0']["PlanningRequest"]["m_origin"]
        except:
            m_pathOrigin = [0,0]    
            
        ego_info = data['Frames']['0']["PlanningRequest"]["m_startPosture"]["m_pose"] ### x,y, theta
        ego_info = [ego_info[0]+m_pathOrigin[0]-nfm_origin[0],ego_info[1]+m_pathOrigin[1]-nfm_origin[1],ego_info[2]]
        try:
            target_info = data['Frames']['0']["PlanningRequest"]["m_targetArea"]["m_targetPosture"]["m_pose"]
        except:
            target_info = data['Frames']['0']["PlanningRequest"]["m_targetAreas"]["m_targetPosture"][0]["m_pose"]  
        target_info = [target_info[0]+m_pathOrigin[0]-nfm_origin[0],target_info[1]+m_pathOrigin[1]-nfm_origin[1],target_info[2]]    
        obstacles = []
        trans_matrix = np.array([[np.cos(target_info[2]), np.sin(target_info[2]), -target_info[0]*np.cos(target_info[2])-target_info[1]*np.sin(target_info[2])],
                            [-np.sin(target_info[2]), np.cos(target_info[2]), target_info[0]*np.sin(target_info[2])-target_info[1]*np.cos(target_info[2])],
                            [0, 0, 1]], dtype=np.float32) 
        for ele in data['Frames']['0']["NfmAggregatedPolygonObjects"]:  # list of dicts
            try: 
                list_nodes = ele["nfmPolygonObjectNodes"]
                assert len(list_nodes) >= 2, "nfmPolygonObjectNodes number is less than 2"
                tmp = []
                last_kk = None
                for i, kk in enumerate(list_nodes):
                    current_kk_x = kk["m_x"]
                    current_kk_y = kk["m_y"]
                    current_kk = [current_kk_x, current_kk_y]
                    
                    if last_kk == None:
                        tmp.append(current_kk)

                    if last_kk is not None:
                        last_kk_x, last_kk_y = last_kk
                        dist = np.sqrt((current_kk_x - last_kk_x) ** 2 + (current_kk_y - last_kk_y) ** 2)

                        if dist > interp_dist:
                            num_points = int(dist // interp_dist)
                            x_values = np.linspace(last_kk_x, current_kk_x, num_points + 2)[1:-1]
                            y_values = np.linspace(last_kk_y, current_kk_y, num_points + 2)[1:-1]

                            for x, y in zip(x_values, y_values):
                                interp_point = [x, y]
                                if not tmp or np.linalg.norm(np.array(interp_point) - np.array(tmp[-1])) > 1e-6:
                                    tmp.append(interp_point)

                        if not tmp or np.linalg.norm(np.array(current_kk) - np.array(tmp[-1])) > 1e-6:
                            tmp.append(current_kk)

                    last_kk = current_kk

                obstacles.append(tmp)  

            except Exception as e:
                print(f'{filename} does not have obstacles or error: {e}')
                            
        flat_list = [item for sublist in obstacles for item in sublist]

        filtered_flat_list = []

        for pt in flat_list:
            x, y = pt
            pt_np = np.array([[x], [y], [1]])  # shape: (3, 1)
            transformed = np.matmul(trans_matrix, pt_np)  # shape: (3, 1)
            x_t, y_t = transformed[0, 0], transformed[1, 0]

            if not (vehicle_x_min <= x_t <= vehicle_x_max and
                    vehicle_y_min <= y_t <= vehicle_y_max):
                filtered_flat_list.append([x, y])
            
        ego_info[2] = normalize_angle(ego_info[2])   ### normalize the heading angle
        target_info[2] = normalize_angle(target_info[2])         
        return ego_info, target_info, np.array(filtered_flat_list, dtype=np.float32), obstacles 