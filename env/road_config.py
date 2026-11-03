"""도로 / 차량 / 배경 교통류 설정 파일.

이 파일의 값을 수정한 뒤 train.py를 다시 실행하면
road_builder.py가 SUMO 도로 파일(.net.xml 등)을 자동으로 다시 만들어준다.
즉, 도로를 바꾸고 싶으면 이 파일만 고치면 된다.

단위 참고:
    - 거리: m (미터)
    - 속도: m/s  (참고: 33.33 m/s ≈ 120 km/h,  27 m/s ≈ 97 km/h)
"""

# ──────────────────────────────────────────────────────────────
# 1) 도로 형상
#    미리보기: SADANG_MANUAL 수작업 경로 / 기존 학습 설정: ROAD.
# ──────────────────────────────────────────────────────────────
# 양재 → 선암TG → 사당IC → 사당역 → 이수 수작업 모형.
# 좌표상 북쪽은 이수역, 남쪽은 사당IC, 동서쪽은 남부순환로 방향.
SADANG_MANUAL = {
    "network": "sadang_manual",
    "yangjae_only": True,  # 선암·사당 구간은 원본에 보관하고 양재IC만 실행
    "arm_length": 350,           # 사거리 중심에서 각 방향 끝까지 거리(m)
    "gangnam_lanes": 3,              # 한 방향 본선 차로 수 (임시)
    "gangnam_speed": 22.22,         # 본선 임시 80km/h
    "gangnam_through_period": 8,    # 본선 계속 주행 차량의 유입 간격(초)
    "preview_focus": "gyeongbu_extension",  # 양재IC와 경부 서울방향 연장부
    # 차로 순서와 대기 시간은 조정 가능한 시뮬레이션 가정이다.
    # SUMO 차로 번호는 오른쪽부터 0이다.
    "seonam_toll": {"cash_lanes": (0, 1, 2), "wait_seconds": 3.0,
                    "hipass_speed": 8.33, "island_length": 7.0},
    # 중앙 통과도로와 바깥 연결도로: 편도 2차로씩, 경계 간격 0.5m.
    # 서로 다른 edge이므로 인접해도 차로 변경할 수 없다.
    "yangjae_parallel_gap": 0.5,
    "yangjae_underpass": {
        "wall_width": 0.6, "portal_half_length": 22.0,
        # 사용자가 지정한 직선 진입부. 경부 교량과 별개의 지하 구간.
        "entry_cover_x": (5270.0, 5360.0),
    },
    # 진입부는 직전 공유 차로 변경 전 상태로 복구한다.
    # 이전 사옥 방향의 직각 연결부 대신 지하차도 진입 직선부터 시작한다.
    "yangjae_approach": {
        "nodes": {
            "yangjae_start": (5652.3,-1145.5,0),
            "heolleung_split": (5533.5,-1242.7,0),
            "yangjae_link": (5423.8,-1339.6,0),
            "sn_main_east": (5657,-1151,0),
            "sn_return_end": (5662.2,-1157.1,0),
            "yj_reverse_east": (5430.7,-1363.2,0),
            "heolleung_return_join": (5552.8,-1259.8,0),
            "heolleung_return_end": (5667.2,-1162.9,0),
        },
        "shapes": {
            "seonam_main_in": [(5657,-1151,0),(5543,-1248,0),
                (5467,-1313,-1),(5375,-1390,-6),(5314,-1442,-6),
                (5083,-1632,-6),(4898,-1794,-6),(4817,-1862,-5),
                (4715,-1946,-2),(4678,-1966,0),(4640,-1981,0),
                (4595,-1994,0),(4514,-2001,0),(4360,-1986,1),
                (4240,-1976,3),(4128,-1968,5),(4015,-1960,7),(3915,-1949,8)],
            "yangjae_central_return": [(4901,-1805,-6),(5089,-1642,-6),
                (5318,-1449,-6),(5380,-1396,-6),(5472,-1319,-1),
                (5548,-1254,0),(5662.2,-1157.1,0)],
            "yj_reverse_c": [(5147,-1615,0),(5310,-1465,0),(5430.7,-1363.2,0)],
            "yangjae_ramp_return": [(5430.7,-1363.2,0),(5552.8,-1259.8,0)],
            "heolleung_return": [(5552.8,-1259.8,0),(5667.2,-1162.9,0)],
        },
    },
    # 지하차도 출구와 IC 측도가 만난 뒤에는 별도 평행도로 없이 편도 4차로.
    # 사진의 삼거리: 본선 양방향 + 북서쪽 매헌로 한 가지만 남긴다.
    "maeheon_corridor": {
        "east": (4994,-1715,0), "junction": (4800,-1860,0),
        "side_end": (4670,-1720,0),
        "west_shape": [(4800,-1860,0),(4705,-1940,0),(4662,-1961,0),(4610,-1980,0)],
        "lanes": 4, "side_lanes": 1,
        "after_t_lanes": 7,  # 사용자 지정: 삼거리 이후 편도 7차로씩
        "align_outer_edges": True,  # 7차로 바깥 경계를 기준으로 배치, 이동량은 median_scale 적용
        "median_scale": 0.5,  # 양 방향 사이 간격을 이전의 절반으로 축소
        "crossroad": (4705,-1940,0),
        "cross_north_end": (4535,-1740,0),
        "cross_south_end": (4875,-2140,0),
        "cross_side_lanes": 2,
    },
    # 2026-06 카카오 로드뷰 분기 표지판 대조. 오른쪽 차로부터 3+1+1+2.
    "seonam_layout": {
        # 모든 병행 도로가 공유하는 기준선. 숫자는 차로 중심의 횡방향 거리(m).
        "axis": [(3105,-1640),(3458,-1818),(3659,-1896),
                 (3915,-1949),(4313,-1966),(4610,-1980)],
        "return_offset": -6.25, "return_surface_offset": -15.25,
        "gwacheon_offset": 21.75,
    },
    "seonam_cleanup": {
        # 사용자 지정 삭제: 서쪽의 교차도로 묶음과 끝이 튀어나온 잔가지.
        "remove_edges": ["umyeon_south","umyeon_north","umyeon_south_before",
            "umyeon_south_after","umyeon_north_before","umyeon_north_after",
            "umyeon_exit","umyeon_entry","umyeon_yangjae_exit",
            "seonam_surface_ramp","seonam_underpass_w","seonam_underpass_e",
            "seonam_banpo_ground","seonam_local_entry","seonam_return_exit"],
        "gwacheon_end": (3530,-1967,0),
    },
    "seonam_split": {
        "surface_lanes": [0,1,2], "toll_entry_lanes": [3],
        "gwacheon_lanes": [4], "toll_main_lanes": [5,6],
        "surface_label": "Banpo / Umyeonsan",
        "gwacheon_label": "Anyang / Gwacheon / Bongdam (47)",
        "toll_label": "Seonam TG / Sadang IC",
        "banpo_end": (3230,-1550,0),
        "toll_main_shape": [(4260,-1967,1),(4140,-1970,4),(4015,-1960,7)],
        "toll_entry_shape": [(4269,-1955,1),(4140,-1945,4),(4015,-1940,7)],
        "gwacheon_shape": [(4270,-1960,-3),(4230,-1962,-6),(4140,-1990,-6),
                            (3880,-1995,-6),(3640,-1980,0),(3470,-1960,0),(3380,-1945,0)],
        "surface_shape": [(4238,-1935,0),(4043,-1918,0),(3944,-1920,0),
                           (3736,-1919,0),(3627,-1910,0),(3520,-1910,0)],
        "banpo_shape": [(3430,-1880,0),(3390,-1800,0),(3310,-1630,0)],
    },
    # 2D 화면에서도 본선 두 개의 높이 차이를 구분한다. 램프는 중간색으로 표시.
    "level_colors": {"lower": "55,55,55", "upper": "68,105,145"},
    "background_color": "51,128,51",  # 기존 SUMO real world 배경 유지
    "lane_widths": {"motorway": 3.5, "arterial": 3.25, "ramp": 3.25, "toll": 3.25},
    "curve_rounding": 18.0,  # 선형 표본의 모서리를 둥글게 할 최대 거리(m)
    # 사당역 사거리=(0,0), x 동쪽 / y 북쪽, 단위 m.
    # 카카오맵의 구조와 OSM 공개 도로 선형을 참고한 수작업 근사치.
    # 차로별 연결, 정확한 노면 형상, 거리와 단속정보는 미검증.
    "ic_geometry": {
        "south_waypoint": (10, -265),
        "merge": (142, -645),
        "south_road_end": (260, -1000),
        "split": (547, -743),
        "exit_prepare": (800, -770),
        "west_end": (88, -787),
        "east_end": (3105, -1640),
        "east_return": (3088, -1670),
        "west_return": (88, -820),
        "ramp_shape": [(547,-743), (525,-728), (490,-718), (440,-713), (320,-720), (270,-730),
                       (245,-738), (220,-730), (195,-710), (142,-645)],
        "highway_shape": [(3105,-1640), (2307,-1227), (1820,-1000),
                          (1250,-840), (800,-770), (547,-743)],
        "highway_return_shape": [(88,-820),(541,-773),(651,-776),(1122,-855),
                                  (1629,-977),(1909,-1075),(2179,-1201),(3088,-1670)],
        "south_shape": [(142,-645), (85,-551), (53,-419), (10,-265)],
        # 현대차 양재 사옥 근처 헌릉로에서 양재대로·선암 방향으로 이어지는 초안.
        # 출발점은 회사 내부가 아닌 공공도로이며 실제 연결의 단순 근사치.
        "yangjae_start": (5852, -1781),
        "heolleung_split": (5612, -1624),
        "yangjae_link": (5390, -1358),
        "yangjae_fork": (5286, -1448),
        "yangjae_merge": (5141, -1570),
        "yangjae_daero_end": (4313, -1966),
        # 경부 서울방향 본선과 양재대로 진출입 연결: OSM 도로망 좌표를 단순화.
        "gyeongbu_south": (5270,-1816),
        "gyeongbu_mid": (5085,-1564),
        "gyeongbu_north": (4931,-1354),
        "gyeongbu_end": (4450,-675),  # 서초 방향으로 기존 본선을 약 400m 연장
        # 경부 반대 방향도 분리해 도로 폭과 양방향 흐름을 표현한다.
        "gyeongbu_southbound_start": (4437,-689),  # 반대 방향도 나란히 연장
        "gyeongbu_southbound_end": (5514,-2199),
        # OSM 연결로 선형 참고. 실제 높이 수치가 아닌 입체교차 구분용 임시값.
        "gyeongbu_height": 8.0,
        "yangjae_to_gyeongbu_shape": [(5286,-1448), (5266,-1458), (5239,-1468),
                                      (5208,-1471), (5186,-1469), (5047,-1437),
                                      (5011,-1424), (4989,-1412), (4931,-1354)],
        "gyeongbu_to_yangjae_shape": [(5085,-1564), (5059,-1509), (5058,-1491),
                                      (5068,-1469), (5084,-1459), (5104,-1458),
                                      (5154,-1470), (5170,-1484), (5176,-1504),
                                      (5174,-1523), (5161,-1543), (5141,-1570)],
        "yangjae_surface_return_shape": [(5147,-1615),(5172,-1588),(5262,-1512),
                                         (5278,-1499),(5314,-1468),(5361,-1427),(5382,-1412)],
    },
    "heolleung_lanes": 2,  # 직전 진입부 복구: 중앙 2+2, 측도 2+2
    "yangjae_link_lanes": 2,  # 연결로 차로 수 (지도 자료 기반, 현장 검증 전)
    "gyeongbu_lanes": 5,      # 경부 서울 방향 본선 (OSM 기반, 현장 검증 전)
    # 양재IC 나머지 고리형 램프의 편집 가능한 좌표. 현장 치수는 미검증.
    "yangjae_loop_nodes": {
        "yangjae_west_fork": (5060,-1632), "g_south_mid": (5066,-1576),
        "g_south_lower": (5155,-1695), "g_north_lower": (5152,-1654),
        "yj_reverse_west": (4930,-1785), "yj_reverse_mid": (5044,-1700),
        "yj_reverse_fork": (5147,-1615), "yj_reverse_east": (5382,-1412),
    },
    "yangjae_loop_shapes": {
        "loop_nw": [(5060,-1632),(5002,-1659),(4982,-1656),(4966,-1643),
                    (4958,-1607),(4963,-1555),(4974,-1538),(4993,-1531),(5026,-1538),(5066,-1576)],
        "loop_sw": [(5155,-1695),(5165,-1739),(5160,-1766),(5137,-1780),
                    (5122,-1781),(5077,-1772),(5055,-1763),(5043,-1751),(5037,-1733),(5044,-1700)],
        "loop_se": [(5147,-1615),(5208,-1572),(5226,-1572),(5243,-1582),
                    (5255,-1610),(5252,-1665),(5236,-1690),(5218,-1696),(5191,-1689),(5152,-1654)],
    },
    # 카카오맵 위성지도와 공개 도로 선형을 대조한 선암 구간 수작업 모형.
    # 선암IC(우면산로)와 선암TG(강남순환로)는 별개. z는 층 구분용 임시값.
    # 양재대로에서 강남순환로로 들어가는 1차로 램프와 2차로 본선이 합류한다.
    "seonam": {
        # 분할 끝점 sn_prepare를 넘어간 좌표를 넣으면 도로가 되꺾인다.
        "upstream_shape": [(5060,-1632),(4934,-1742),(4860,-1807),(4751,-1904),
                           (4705,-1940),(4662,-1961),(4610,-1980)],
        "nodes": {
            "sn_prepare": (4610,-1980,0),
            "sn_merge": (3915,-1949,8), "sn_main_east": (5314,-1442,0),
            "sn_tg_fan": (3659,-1896,8), "sn_tg_gate_in": (3550,-1856,8),
            "sn_tg_gate_out": (3505,-1837,8), "sn_tg_rejoin": (3387,-1785,8),
            "sn_return_split": (3916,-1962,8), "sn_return_merge": (4335,-1998,8),
            "sn_return_tunnel_end": (4901,-1805,-6), "sn_return_end": (5318,-1449,0),
            "sn_local_w": (3458,-1918,0), "sn_local_ramp": (3348,-1936,0),
            "sn_local_end": (3240,-2003,0),
            "sn_local_in": (3200,-2049,-6), "sn_local_bridge": (3615,-1943,0),
            "sn_local_e": (4000,-1991,0), "sn_local_merge": (4338,-2011,0),
            "sn_local_east_end": (4550,-2035,0),
            "um_north_in": (3255,-1668,-6), "um_south_out": (3424,-2141,-6),
            "um_south_in": (3436,-2151,-6), "um_north_out": (3273,-1662,-6),
            "um_outer_n_in": (3160,-1416,-6), "um_outer_n_out": (3205,-1475,-6),
            "um_outer_s_out": (3431,-2230,0), "um_outer_s_in": (3441,-2230,0),
            "heolleung_return_join": (5642,-1674,0), "heolleung_return_end": (5856,-1801,0),
        },
        # id, 시작점, 끝점, 차로 수, 속도(m/s), 선형(x,y,z). None은 직선.
        "edges": [
            # 2026-06 로드뷰: 분기 직전 3차로에서 왼쪽 1차로만 고가 진입.
            # 오른쪽 2차로는 지상 양재대로로 계속 간다(SUMO 오른쪽=0).
            ("seonam_lane_prepare", "sn_prepare", "yangjae_daero_end", 3, 16.67,
             [(4610,-1980,0),(4494,-1983,0),(4400,-1974,0),(4313,-1966,0)]),
            ("seonam_approach", "yangjae_daero_end", "sn_merge", 1, 16.67,
             [(4313,-1966,0),(4269,-1965,1),(4090,-1952,4),(4016,-1950,6),(3915,-1949,8)]),
            ("seonam_main_in", "sn_main_east", "sn_merge", 2, 22.22,
             [(5314,-1442,0),(5083,-1632,0),(4898,-1794,0),(4817,-1862,0),
              (4715,-1946,0),(4678,-1966,0),(4640,-1981,0),(4595,-1994,0),
              (4514,-2001,0),(4360,-1986,1),(4240,-1976,3),(4128,-1968,5),
              (4015,-1960,7),(3915,-1949,8)]),
            ("seonam_main", "sn_merge", "sn_tg_fan", 3, 22.22,
             [(3915,-1949,8),(3867,-1944,8),(3773,-1926,8),(3659,-1896,8),
              ]),
            # 선암TG 통행구: 로드뷰의 하이패스 3개 + 현금 통행구 3개.
            # 통행구 형상만 반영하며 운영 중인 부스와 요금 처리시간은 별도다.
            ("seonam_tg_fan", "sn_tg_fan", "sn_tg_gate_in", 6, 13.89,
             [(3659,-1896,8),(3600,-1873,8),(3550,-1856,8)]),
            ("seonam_tg_gate", "sn_tg_gate_in", "sn_tg_gate_out", 6, 8.33, None),
            ("seonam_tg_rejoin", "sn_tg_gate_out", "sn_tg_rejoin", 6, 13.89, None),
            ("seonam_after_tg", "sn_tg_rejoin", "gangnam_east", 3, 22.22,
             [(3387,-1785,8),(3174,-1676,8),(3105,-1640,8)]),
            ("seonam_return", "gangnam_east_return", "sn_return_split", 3, 22.22,
             [(3088,-1670,8),(3184,-1719,8),(3365,-1799,8),(3514,-1859,8),
              (3655,-1908,8),(3788,-1942,8),(3916,-1962,8)]),
            ("seonam_return_main", "sn_return_split", "sn_return_merge", 2, 22.22,
             [(3916,-1962,8),(4037,-1970,8),(4127,-1977,8),(4239,-1989,8),(4335,-1998,8)]),
            # 반대편 중앙 본선도 매헌지하차도까지 이어서 임의의 짧은 끝을 없앤다.
            ("seonam_return_east", "sn_return_merge", "sn_return_tunnel_end", 3, 13.89,
             [(4335,-1998,8),(4466,-2007,2),(4515,-2011,-3),(4594,-2003,-6),
              (4678,-1977,-6),(4724,-1954,-6),(4901,-1805,-6)]),
            ("yangjae_central_return", "sn_return_tunnel_end", "sn_return_end", 2, 13.89,
             [(4901,-1805,-6),(5089,-1642,-3),(5318,-1449,0)]),
            ("seonam_surface_w", "yangjae_daero_end", "sn_local_w", 2, 16.67,
             [(4313,-1966,0),(4238,-1953,0),(4043,-1936,0),(3944,-1938,0),
              (3736,-1937,0),(3627,-1928,0),(3520,-1918,0),(3458,-1918,0)]),
            ("seonam_surface_ramp", "sn_local_w", "sn_local_ramp", 2, 13.89,
             [(3458,-1918,0),(3415,-1920,0),(3393,-1924,0),(3348,-1936,0)]),
            ("seonam_underpass_w", "sn_local_ramp", "sn_local_end", 2, 13.89,
             [(3348,-1936,0),(3316,-1950,0),(3275,-1976,0),(3240,-2003,0)]),
            ("seonam_underpass_e", "sn_local_in", "sn_local_bridge", 2, 13.89,
             [(3200,-2049,-6),(3253,-2005,-9),(3308,-1971,-12),(3344,-1955,-12),
              (3382,-1944,-12),(3423,-1936,-10),(3467,-1931,-7),(3525,-1936,-4),(3615,-1943,0)]),
            ("seonam_surface_e", "sn_local_bridge", "sn_local_e", 2, 16.67,
             [(3615,-1943,0),(3773,-1961,0),(3928,-1978,0),(4000,-1991,0)]),
            ("seonam_surface_e2", "sn_local_e", "sn_local_merge", 2, 16.67,
             [(4000,-1991,0),(4036,-1995,0),(4091,-1999,0),(4142,-2003,0),
              (4191,-2006,0),(4249,-2009,0),(4338,-2011,0)]),
            ("seonam_surface_out", "sn_local_merge", "sn_local_east_end", 2, 16.67, None),
            ("seonam_local_to_yangjae", "sn_local_east_end", "yj_reverse_west", 2, 16.67,
             [(4550,-2035,0),(4640,-2018,0),(4710,-1985,0),(4770,-1923,0),
              (4864,-1850,0),(4930,-1785,0)]),
            ("seonam_return_exit", "sn_return_split", "sn_local_merge", 1, 16.67,
             [(3916,-1962,8),(4010,-1979,8),(4126,-1991,7),(4237,-2003,4),(4338,-2011,0)]),
            ("seonam_local_entry", "sn_local_e", "sn_return_merge", 1, 16.67,
             [(4000,-1991,0),(4033,-1987,0),(4070,-1986,0),(4120,-1988.5,0),
              (4229,-1994,4),(4335,-1998,8)]),
            ("umyeon_south", "um_north_in", "um_south_out", 2, 19.44,
             [(3255,-1668,-6),(3274,-1715,-6),(3332,-1877,-6),(3372,-1979,-6),(3424,-2141,-6)]),
            ("umyeon_north", "um_south_in", "um_north_out", 2, 19.44,
             [(3436,-2151,-6),(3429,-2120,-6),(3400,-2033,-6),(3344,-1873,-6),(3273,-1662,-6)]),
            ("umyeon_south_before", "um_outer_n_in", "um_north_in", 2, 19.44,
             [(3160,-1416,-6),(3186,-1484,-6),(3220,-1583,-6),(3249,-1654,-6),(3255,-1668,-6)]),
            ("umyeon_south_after", "um_south_out", "um_outer_s_out", 2, 19.44,
             [(3424,-2141,-6),(3431,-2186,-3),(3431,-2230,0)]),
            ("umyeon_north_before", "um_outer_s_in", "um_south_in", 2, 19.44, None),
            ("umyeon_north_after", "um_north_out", "um_outer_n_out", 2, 19.44,
             [(3273,-1662,-6),(3251,-1594,-6),(3243,-1575,-6),(3205,-1475,-6)]),
            ("umyeon_exit", "um_north_in", "sn_local_end", 1, 13.89,
             [(3255,-1668,-6),(3261,-1708,-6),(3297,-1834,-6),(3308,-1885,-5),
              (3303,-1917,-4),(3280,-1953,-2),(3240,-2003,0)]),
            ("umyeon_entry", "sn_local_ramp", "um_south_out", 1, 13.89,
             [(3348,-1936,0),(3336,-1960,0),(3334,-1980,0),(3350,-2012,-1),
              (3403,-2093,-4),(3424,-2141,-6)]),
            ("umyeon_yangjae_exit", "um_south_in", "sn_local_bridge", 1, 13.89,
             [(3436,-2151,-6),(3441,-2133,-6),(3433,-2077,-6),(3422,-2023,-6),
              (3421,-1986,-6),(3440,-1956,-5),(3476,-1942,-4),(3615,-1943,0)]),
            ("yangjae_ramp_return", "yj_reverse_east", "heolleung_return_join", 2, 13.89,
             [(5382,-1412,0),(5391,-1411,0),(5408,-1430,0),(5428,-1452,0),
              (5541,-1591,0),(5564,-1617,0),(5590,-1641,0),(5602,-1648,0),(5642,-1674,0)]),
            ("heolleung_return", "heolleung_return_join", "heolleung_return_end", 5, 13.89,
             [(5642,-1674,0),(5656,-1685,0),(5684,-1704,0),(5710,-1721,0),(5856,-1801,0)]),
        ],
        "connections": [
            ("seonam_lane_prepare","seonam_approach",2,0),
            ("seonam_lane_prepare","seonam_surface_w",0,0),
            ("seonam_lane_prepare","seonam_surface_w",1,1),
            ("seonam_approach","seonam_main",0,0),
            ("seonam_main_in","seonam_main",0,1), ("seonam_main_in","seonam_main",1,2),
            ("seonam_return","seonam_return_exit",0,0),
            ("seonam_return","seonam_return_main",1,0), ("seonam_return","seonam_return_main",2,1),
            ("seonam_local_entry","seonam_return_east",0,0),
            ("seonam_return_main","seonam_return_east",0,1), ("seonam_return_main","seonam_return_east",1,2),
            ("seonam_return_exit","seonam_surface_out",0,1),
            ("seonam_surface_e2","seonam_surface_out",0,0), ("seonam_surface_e2","seonam_surface_out",1,1),
            ("seonam_surface_e","seonam_local_entry",0,0),
            ("seonam_surface_ramp","umyeon_entry",0,0),
            ("umyeon_yangjae_exit","seonam_surface_e",0,0),
            ("seonam_return_east","yangjae_central_return",0,0),
            ("seonam_return_east","yangjae_central_return",1,1),
            ("seonam_return_east","yangjae_central_return",2,1),
            ("seonam_surface_out","seonam_local_to_yangjae",0,0),
            ("seonam_surface_out","seonam_local_to_yangjae",1,1),
            ("seonam_local_to_yangjae","yj_reverse_a",0,0),
            ("seonam_local_to_yangjae","yj_reverse_a",1,1),
            ("umyeon_south_before","umyeon_south",0,0),
            ("umyeon_south_before","umyeon_south",1,1),
            ("umyeon_south_before","umyeon_exit",0,0),
            ("umyeon_south","umyeon_south_after",0,0),
            ("umyeon_south","umyeon_south_after",1,1),
            ("umyeon_entry","umyeon_south_after",0,0),
            ("umyeon_north_before","umyeon_north",0,0),
            ("umyeon_north_before","umyeon_north",1,1),
            ("umyeon_north_before","umyeon_yangjae_exit",0,0),
            ("umyeon_north","umyeon_north_after",0,0),
            ("umyeon_north","umyeon_north_after",1,1),
            ("yj_reverse_c","yangjae_ramp_return",0,0),
            ("yj_reverse_c","yangjae_ramp_return",1,1),
            ("yangjae_ramp_return","heolleung_return",0,0),
            ("yangjae_ramp_return","heolleung_return",1,1),
        ],
        "fixed_lane_edges": ["seonam_tg_gate"],
        "route_expansions": {
            "yangjae_daero_d": ["yangjae_daero_d", "seonam_lane_prepare"],
            "seonam_main": ["seonam_main", "seonam_tg_fan", "seonam_tg_gate",
                            "seonam_tg_rejoin", "seonam_after_tg"],
            "gangnam_in": ["gangnam_in", "gangnam_exit_prepare"],
            "seonam_return_east": ["seonam_return_east", "yangjae_central_return"],
            "seonam_surface_out": ["seonam_surface_out", "seonam_local_to_yangjae",
                                   "yj_reverse_a", "yj_reverse_b", "yj_reverse_c"],
            "umyeon_south": ["umyeon_south_before", "umyeon_south", "umyeon_south_after"],
            "umyeon_north": ["umyeon_north_before", "umyeon_north", "umyeon_north_after"],
            "umyeon_exit": ["umyeon_south_before", "umyeon_exit"],
            "umyeon_entry": ["umyeon_entry", "umyeon_south_after"],
            "umyeon_yangjae_exit": ["umyeon_north_before", "umyeon_yangjae_exit"],
            "yj_reverse_c": ["yj_reverse_c", "yangjae_ramp_return", "heolleung_return"],
        },
    },
    "junction_radii": {"yj_reverse_east": 14},
    "lanes": {"north": 4, "south": 4, "east": 4, "west": 4},
    # 사당역 북쪽 로드뷰: 일반차로 3개 + 중앙 버스차로 1개(북진 방향).
    # 중간 구간 반대 방향도 로드뷰로 대조. 사거리 남쪽 전 구간은 별도다.
    "city_bus_lanes": {"north_out": 3, "isu_out": 3, "north_in": 3, "isu_in": 3,
                       "isu_finish_out": 3, "isu_finish_in": 3},
    "city_geometry": {
        "north": (16,350), "isu_cross": (58,920), "isu_finish": (79,1147),
        "isu_west": (-188,827), "isu_east": (245,943),
        "north_shape": [(0,0),(6,130),(13,268),(16,350)],
        "isu_shape": [(16,350),(24,486),(34,623),(45,757),(57,905),(58,920)],
        "finish_shape": [(58,920),(62,970),(69,1038),(79,1147)],
        "west_shape": [(-188,827),(-86,867),(-35,890),(15,914),(58,920)],
        "east_shape": [(245,943),(180,940),(120,931),(58,920)],
    },
    # 분기/합류부 차로 증감은 정지 교차로가 아니라 긴 연결 면으로 처리한다.
    "edge_end_offsets": {"seonam_main": 45, "seonam_tg_rejoin": 55,
                         "yangjae_daero_d": 25, "gangnam_in": 30},
    "speed_limit": 13.89,        # 임시 50km/h, 실제 제한속도 확인 필요
    "flow_period": 12,           # 접근 방향마다 차량 한 대씩 나오는 간격(초)
}

ROAD = {
    "network": "yangjae_isu",  # OSM 도로 전용 미리보기 (학습환경은 별도 확장 필요)
    "preview_traffic": True,   # 일반 SUMO 차량: 초기 배치 + 시간당 600대 유입
    "preview_view_width": 320.0,  # 사당역 주변 차로 연결 확인용 화면 폭(m)
    "length": 7448.74,    # 선택 경로의 외부 edge 길이 합계 (교차로 내부 제외)
                           # 스텝당 0.5초 × 평균 24 m/s 기준 약 1000스텝 분량.
                           # (최고속도 33.3 m/s로 내내 달리면 약 720스텝에 완주)
    "num_lanes": 4,        # 차선 수. 관측의 차선 정규화에도 쓰임
    "speed_limit": 33.33,  # 도로 제한속도 (m/s). ego의 이론상 최고속도이자
                           # 관측/보상에서 속도를 정규화하는 기준(vmax)
}

# ──────────────────────────────────────────────────────────────
# 2) ego 차량 (RL 에이전트가 조종하는 빨간 차)
#    accel/decel은 SUMO 물리엔진이 허용하는 "한계치"이고,
#    실제 한 스텝에 얼마나 가감속할지는 mdp_config.py의
#    SPEED_DELTAS(행동 정의)가 결정한다.
# ──────────────────────────────────────────────────────────────
# ──────────────────────────────────────────────────────
# 도로변 장식 (순수 시각 요소 — 시뮬레이션 물리/학습에 영향 없음)
# GUI에서 도로가 허허벌판에 떠 있는 느낌을 줄이고,
# ego 추적 화면에서 "지나가는 배경"이 생겨 속도감이 보인다.
# ──────────────────────────────────────────────────────
SCENERY = {
    "trees": True,          # 나무 심기 on/off
    "tree_spacing": 45.0,   # 같은 쪽 나무 사이 평균 간격 (m)
    "tree_jitter": 0.5,     # 간격/위치의 무작위 흔들림 비율
    "side_offset": 6.0,     # 도로 가장자리에서 나무까지 거리 (m)
    "seed": 7,              # 배치 난수 시드 (매번 같은 숲)
}

EGO = {
    "accel": 5.4,          # 최대 가속도 (m/s^2)  → 행동 a=+1 일 때
    "decel": 5.4,          # 최대 감속도 (m/s^2)  → 행동 a=-1 일 때
    "max_speed": 33.33,    # 차량 자체의 최고속도
    "depart_lane": "center", # 시작 차선. "center"면 중앙 차선(차선수//2)에서
                           # 자동 시작 — 차선 수를 바꿔도 항상 가운데.
    "depart_speed": 10,    # 시작 속도 (m/s)
}

# ──────────────────────────────────────────────────────────────
# 3) 배경 교통류 (SUMO가 알아서 운전하는 일반 차량들)
#    ego보다 느리게 설정해서 "느린 앞차를 만나면 감속해야 하는"
#    상황이 자연스럽게 만들어지도록 했다.
# ──────────────────────────────────────────────────────────────
TRAFFIC = {
    # ══════════════════════════════════════════════════════
    # ★ 추월 학습 시나리오 ★
    #
    # 배경차는 ego(최고 33.33 m/s)보다 확연히 느리게(14~19 m/s) 달리고,
    # 컨트롤러마다 희망속도를 다르게 줘서 차선/차량 간 속도차를 만든다.
    # → 같은 차선에 계속 있으면 느린 앞차에 막히고,
    #   빈 차선으로 변경해야 속도 보상을 더 받을 수 있는 구조.
    #   즉 "차선변경을 통한 추월"이 최적 정책이 되도록 설계된 환경.
    #
    # 밀도는 1500대/시간: 추월할 틈은 있되, 한 차선이 계속 비어있진
    # 않은 수준. (너무 높으면 전 차선이 막혀 추월 자체가 불가능해지고,
    # 너무 낮으면 앞차를 만날 일이 없어 차선변경을 배울 이유가 없다)
    # ══════════════════════════════════════════════════════
    "max_speed": 16.0,        # 배경차 공통 기본 최고속도 (m/s ≈ 58 km/h)
                              #   ego 33.33의 절반 수준 → 추월 이득이 큼
    "vehs_per_hour": 3000,    # 시간당 투입 대수. 1500→1200 완화:
                              #   전체가 꽉 막힌 느낌을 줄이고
                              #   차선 사이 속도차/틈이 살아나게 한다
    "depart_lane": "random",
    "depart_speed": 8.0,      # 신규 차량 진입 속도 (m/s).
                              # 입구에서 천천히 들어와 자기 컨트롤러로 가속.
                              # "random"이면 고속 스폰 차량이 입구 부근에서
                              # 다른 차와 얽히기 쉬워 고정 저속으로 둔다.

    # ──────────────────────────────────────────────────────
    # 배경차 컨트롤러(차량추종모델) 믹스
    #
    # 실제 도로에는 운전 스타일이 제각각인 차들이 섞여 있다. 배경차를
    # 단일 모델로만 채우면 에이전트가 그 모델의 버릇에 과적합되므로,
    # 서로 다른 SUMO carFollowModel 4종을 확률적으로 섞어 투입한다.
    # (SUMO의 vTypeDistribution 기능 사용 — road_builder.py가 생성)
    #
    # probability: 투입 비율 (합이 1이 아니어도 자동 정규화됨)
    # max_speed  : 컨트롤러별 희망속도 (생략 시 위 공통값).
    #              서로 다르게 → 차선 간 속도차 → 추월 기회 발생
    # speed_dev  : 같은 타입 안에서도 개체별 희망속도 편차 (0~1)
    # color      : GUI에서 구분용 (ego의 빨강은 피할 것)
    # sigma      : Krauss 계열 전용 "운전자 불완전성" (다른 모델은 무시)
    # ──────────────────────────────────────────────────────
    "controllers": {
        "krauss": {   # 산만한 인간 운전자 — 제일 느리고 들쑥날쑥
            "probability": 0.35,
            "carFollowModel": "Krauss",
            "accel": 2.0, "decel": 4.5,
            "tau": 1.0, "min_gap": 2.5,
            "sigma": 0.3,
            "max_speed": 14.0, "speed_dev": 0.15,
            "color": "1,1,0",    # 노랑
        },
        "idm": {      # 부드러운 표준 인간 모델 — 중간 속도
            "probability": 0.30,
            "carFollowModel": "IDM",
            "accel": 1.8, "decel": 3.5,
            "tau": 1.2, "min_gap": 2.0,
            "max_speed": 16.0, "speed_dev": 0.10,
            "color": "0,1,1",    # 시안
        },
        "eidm": {     # 반응지연 있는 현실적 인간 — 중간 속도
            "probability": 0.20,
            "carFollowModel": "EIDM",
            "accel": 2.0, "decel": 4.0,
            "tau": 1.1, "min_gap": 2.0,
            "max_speed": 16.5, "speed_dev": 0.10,
            "color": "1,0.6,0",  # 주황
        },
        "acc": {      # 크루즈 차량 — 배경차 중엔 가장 빠르고 일정함
            "probability": 0.15,
            "carFollowModel": "ACC",
            "accel": 2.0, "decel": 4.0,
            "tau": 1.5, "min_gap": 2.0,
            "max_speed": 19.0, "speed_dev": 0.05,
            "color": "1,0,1",    # 마젠타
        },
    },

    # ──────────────────────────────────────────────────────
    # 프리필(pre-fill): 에피소드 시작 시 배경차를 도로 전체에 미리 배치
    #
    # flow(위의 vehs_per_hour)는 도로 "입구"에서만 차를 만들기 때문에,
    # 그냥 시작하면 ego 앞의 긴 도로(12km)가 텅 비어 있다가 한참 뒤에야
    # 채워진다. 프리필은 reset()마다 도로 전 구간에 차량을 일정 간격
    # (± 무작위 흔들림)으로 미리 심어서, 첫 스텝부터 실제 교통류 속에서
    # 학습이 이뤄지게 한다. 타입은 controllers 비율대로 샘플링되고,
    # 초기 속도는 각 개체의 허용 최고속도(departSpeed="max")로 시작한다.
    # ──────────────────────────────────────────────────────
    "prefill": {
        "enabled": True,
        "start_offset": 80,    # ego 시작점(0m) 앞 이 구간은 비워둠 (진입 공간)
        "end_margin": 50,      # 도로 끝 이 구간도 비워둠
        "spacing": None,       # 차선당 배치 간격(m). None이면
                               # 교통량/속도에서 자동 계산: 속도 / (차선당 유량)
        "spacing_jitter": 0.45,  # 간격의 ±45% 무작위 흔들림.
                                 # 크게 줄수록 차들이 "무리(platoon) + 틈"
                                 # 형태로 뭉쳐서, 틈을 노려 빠져나가는
                                 # 칼치기 주행이 가능한 지형이 된다.
        # 차선별 간격 배율 (인덱스 0 = 맨 오른쪽 차선):
        #   오른쪽은 촘촘(느린 차들이 몰린 주행차로 느낌),
        #   왼쪽은 성기게(추월차로) → 항상 "뚫린 쪽"이 존재해서
        #   차선변경으로 앞서나가는 전략이 구조적으로 성립한다.
        "lane_spacing_factor": [0.85, 1.0, 1.35],
    },
}

