#launch Isaac Sim before any other imports
#default first two lines in any standalone application
import sys
sys.path.append("/home/kimseungjun/.local/share/ov/pkg/isaac_sim-2022.2.0/extension_examples/user_examples")
import omni.isaac.core.utils.bounds as bounds_utils 
import math
import re
from itertools import zip_longest
class CollisionCheck:
    def __init__(self) -> None:
        self.prime_tuple={}
        self.prime_tuple_name={}
        self.prime_list=[]
        self.background_prime=None
    def position_check(self):
        prim_list=self.prime_list[:]
        prim_list.insert(0,self.background_prime)
        position=[]
        orientation=[]
        for i,name in enumerate(prim_list):
            position1, quert = name.get_local_pose()
            # orientation=quat_to_euler_angles(quert)
            position.append(position1)
            orientation.append(quert)
        return position,orientation

    def collistion_check(self,prime,compute_list2):
        target_position=prime.get_local_pose()[0]
        for _, i in enumerate(compute_list2):
            existed_position=i.get_local_pose()[0]
            #print(math.sqrt((existed_position[0]-target_position[0])**2 + (existed_position[1]-target_position[1])**2))
            distance=math.sqrt((existed_position[0]-target_position[0])**2 + (existed_position[1]-target_position[1])**2)
            # print('distance:',distance)
            if distance <= 0.07 and distance != 0.0:
                return False
        return True


    def comput_relationship(self):
        prime_tuple=self.prime_tuple
        prime_tuple_name=self.prime_tuple_name
        prime_list=self.prime_list[:]
        # print('prime_list:',prime_list)
        compute_list=[]
        # compute_list2=[]
        previous_object_list=[]
        prime_list.sort(key=lambda x: x.get_local_pose()[0][2])#z축 기준으로 정렬
        while prime_list:
            state_check=False
            prime=prime_list.pop(0)
            # print('prime:',prime_tuple_name[prime])
            if not compute_list:
                compute_list.append(f"{prime_tuple_name[prime]} is ON KLT_Bin_Box")
                # compute_list2.append(prime)
            # elif self.collistion_check(prime=prime,compute_list2=compute_list2):
            #     compute_list.append(f"from_id: {prime_tuple[prime]}, to_id: 0, relation_type: ON")
            #     compute_list2.append(prime)
            else:
                check_list=[]
                what_the_fuck=previous_object_list[:] #이미 놓았던 object list 얕은복사
                for prime2 in what_the_fuck: #이미 놓았던 object list를 순회
                    state,IoU=self.check_IoU(prime=prime,prime2=prime2)
                    if state:
                        # print('prime2 and prime:',prime_tuple_name[prime2], prime_tuple_name[prime])
                        state_check=True
                        compute_list.append(f"{prime_tuple_name[prime]} is ON {prime_tuple_name[prime2]}")
                if state_check==False:
                    compute_list.append(f"{prime_tuple_name[prime]} is ON KLT_Bin_Box")

                #         check_list.append({prime2:IoU})
                # check_list.sort(key=lambda x: list(x.values()), reverse=True)
                # final_prime=list(check_list[0].keys())[0]
                # compute_list.append(f"from_id: {prime_tuple[prime]}, to_id: {prime_tuple[final_prime]}, relation_type: ON")
            previous_object_list.append(prime)

        return compute_list

    def check_IoU(self,prime,prime2):
        #sugar와 bleach cleanser의 경우는 sugar가 prime2 
        #bleach_cleanser가 prime
        prime_tuple=self.prime_tuple
        prime_tuple_name=self.prime_tuple_name
        # print(prime_tuple_name[prime])
        # print(prime_tuple_name[prime2])
        # print('prime1:',prime_tuple_name[prime])
        # print('prime2:',prime_tuple_name[prime2])
        cache = bounds_utils.create_bbox_cache()
        prime_shape_list=bounds_utils.compute_aabb(cache, prim_path=f"/World/obj/{self.prime_tuple_name[prime]}")
        prime_x_half_length=(prime_shape_list[3]-prime_shape_list[0])/2.0
        prime_y_half_length=(prime_shape_list[4]-prime_shape_list[1])/2.0
        prime2_shape_list=bounds_utils.compute_aabb(cache, prim_path=f"/World/obj/{self.prime_tuple_name[prime2]}")
        prime2_x_half_length=(prime2_shape_list[3]-prime_shape_list[0])/2.0
        prime2_y_half_length=(prime2_shape_list[4]-prime_shape_list[1])/2.0
        position,_=self.position_check()

        prime_number=self.prime_tuple[prime]
        prime2_number=self.prime_tuple[prime2]
        prime_region=[[(position[prime_number][0]-prime_x_half_length),(position[prime_number][0]+prime_x_half_length)],
                     [(position[prime_number][1]-prime_y_half_length),(position[prime_number][1]+prime_y_half_length)]]
        prime2_region=[[(position[prime2_number][0]-prime2_x_half_length),(position[prime2_number][0]+ prime2_x_half_length)],
                      [(position[prime2_number][1]-prime2_y_half_length),(position[prime2_number][1]+prime2_y_half_length)]]
        #check intersection
        intersection_x1=max(prime_region[0][0],prime2_region[0][0])*0.85 ##겹치는 영역 x1 좌측
        intersection_x2=min(prime_region[0][1],prime2_region[0][1])*0.85 #겹치는 영역 x2 우측
        intersection_y1=min(prime_region[1][1],prime2_region[1][1])*0.85 #겹치는 영역 y1 상단
        intersection_y2=max(prime_region[1][0],prime2_region[1][0])*0.85#겹치는 영역 y2 하단
        
        intersection=max(0,intersection_x2-intersection_x1)*max(0,intersection_y1-intersection_y2) 

        box1_area = abs((prime_region[0][1]-prime_region[0][0])*(prime_region[1][1]-prime_region[1][0]))
        box2_area = abs((prime2_region[0][1]-prime2_region[0][0])*(prime2_region[1][1]-prime2_region[1][0]))

        IoU= intersection/(box1_area+box2_area -intersection +1e-7)

        # print('IoU:',IoU)
        
        if IoU < 0.144 :
            return False, IoU
        else:
            return True ,IoU

    def print_output_message(self):
        message=[]
        position,_=self.position_check()
        prime=self.prime_list[:]
        message.append(f'id: 0, category: Box, class_name: KLT_Bin_box obj_transform: position: {position[0]}, ')
        for number, object in enumerate(prime):
            message.append(f'id: {self.prime_tuple[object]}, category: objects, class_name: {self.prime_tuple_name[object]} obj_transform: position: {position[self.prime_tuple[object]]}, ')
        
        message_relationship=self.comput_relationship()
        for i in message_relationship:
            message.append(i+',')
        # message_str="/n ".join(message_relationship)
        # print('message_str',type(message_relationship))
        # message.append(message_str)
        # print(message)
        return message
    
    def make_a_unknown(self):
        list_message=self.print_output_message()
        text_message=''.join(list_message)
        original_node=[no for no in list_message if 'class_name' in no]
        original_relation=[rel for rel in list_message if 'is ON' in rel]
        print('original_node:',original_node)
        print('original_rel:',original_relation) 
        node=re.findall(r"class_name:\s*([^\s\n,]+)", text_message)
        print('node:',node)
        pri_rel=[]
        lat_rel=[]
        relation=[i.split('is ON') for i in original_relation]
        for j in relation:
            pri_rel.append(j[0].strip().rstrip(','))
            lat_rel.append(j[1].strip().rstrip(','))


        # latter_relation=re.findall(r"ON\s*([^\s\n,]+)", text_message)
        del_relation=[el for el in lat_rel if el !='KLT_Bin_Box']
        # prior_relation=re.findall(r"(.+?)\s+is", text_message)
        # print('del_relation:',del_relation)
        # print('prior_relation:',prior_relation)
        dict_node={}
        reverse_dict_node={}
        for i,j in enumerate(node):
            dict_node[i]=j
            reverse_dict_node[j]=i

# #dict_node는 {0: 'KLT_Bin_box', 1: 'cracker_box', 2: 'sugar_box', 3: 'tomato_can', 4: 'gelatin_box', 5: 'meat_can', 6: 'bleach_cleanser', 7: 'pudding_box', 8: 'master_chef_can', 9: 'wood_block', 10: 'pitcher_base'}        
#         # print('dict_node',dict_node)
        replace_node=[]
        replace_rel=[]
        for k,l in zip(node,original_node):
            # print('l',l)
            if k in del_relation:
                # print('여기 실행됨')
                # print(f'{dict_node[k]}')
                a=l.replace(f'{k}', f'<Unknown_object_{reverse_dict_node[k]}>')
                replace_node.append(a)
            else:
                replace_node.append(l)

        print('pri_rel:',pri_rel)
        print('lat_rel:',lat_rel)
        for pri,ori,lat in zip(pri_rel,original_relation,lat_rel):    
            if pri in del_relation:
                # print('여기 실행됨')
                # print(f'{dict_node[k]}')
                b=ori.replace(f'{pri}', f'<Unknown_object_{reverse_dict_node[pri]}>')
                replace_rel.append(b)
                if lat in del_relation:
                    c=ori.replace(f'{lat}', f'<Unknown_object_{reverse_dict_node[lat]}>')
                    replace_rel.append(c)
            elif lat in del_relation:
                c=ori.replace(f'{lat}', f'<Unknown_object_{reverse_dict_node[lat]}>')
                replace_rel.append(c)
            else:
                replace_rel.append(ori)

            
        # for k,l in zip(node,original_relation):
        #     print('l',l)
        #     if k in del_relation:
        #         print('여기 실행됨')
        #         # print(f'{dict_node[k]}')
        #         a=l.replace(f'{k}', f'<Unknown_object_{reverse_dict_node[k]}>')
        #         replace_rel.append(a)
        #     else:
        #         replace_rel.append(l)

        output_list=replace_node+replace_rel
        return output_list