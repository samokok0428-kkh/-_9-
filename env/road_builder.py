"""configs/road_config.py 를 읽어 SUMO 도로 파일들을 자동 생성하는 모듈.

SUMO는 도로를 여러 XML 파일로 정의한다:
    highway.nod.xml  : 노드(교차점/도로 끝점)의 좌표
    highway.edg.xml  : 엣지(도로 구간) — 어느 노드에서 어느 노드로, 차선 수, 제한속도
    highway.net.xml  : 위 두 파일을 netconvert 도구로 컴파일한 "완성된 도로망"
                       (SUMO가 실제로 읽는 파일. 직접 작성하기엔 복잡해서 도구로 생성)
    highway.rou.xml  : 차량 타입(vType)과 교통 수요(flow/route) 정의
    highway.sumocfg  : 위 파일들을 묶어주는 설정 파일 (시뮬레이션 실행의 진입점)

train.py / test.py 가 시작할 때마다 build()를 호출하므로,
road_config.py만 수정하면 도로가 자동으로 다시 만들어진다.
"""
import os
import subprocess

from sumolib import checkBinary  # SUMO 실행파일(sumo, netconvert 등) 경로를 찾아주는 유틸
from env.mdp_config import SIMULATION
try:
    from env.road_config import SCENERY
except ImportError:      # 구버전 설정 하위호환
    SCENERY = {"trees": False}


def build(road: dict, ego: dict, traffic: dict, out_dir: str) -> str:
    """SUMO 파일들을 out_dir에 생성하고, sumocfg 파일 경로를 반환한다.

    Args:
        road:    configs.road_config.ROAD    (도로 형상)
        ego:     configs.road_config.EGO     (RL 차량 속성)
        traffic: configs.road_config.TRAFFIC (배경 교통류)
        out_dir: 생성 위치 (보통 프로젝트의 sumo/ 폴더)
    """
    os.makedirs(out_dir, exist_ok=True)
    if road.get("network") == "sadang_manual":
        return _build_sadang_manual(road, out_dir)
    if road.get("network") == "yangjae_isu":
        return _build_yangjae_isu(road, ego, out_dir)

    # ──────────────────────────────────────────────────────
    # 1) 노드 파일: 직선 도로의 양 끝점 2개
    #    n0 = 원점(출발), n1 = length만큼 떨어진 지점(도착)
    # ──────────────────────────────────────────────────────
    with open(os.path.join(out_dir, "highway.nod.xml"), "w") as f:
        f.write(f"""<nodes>
    <node id="n0" x="0" y="0"/>
    <node id="n1" x="{road['length']}" y="0"/>
</nodes>
""")

    # ──────────────────────────────────────────────────────
    # 2) 엣지 파일: n0 → n1 을 잇는 도로 구간 하나
    #    numLanes = 차선 수, speed = 제한속도(m/s)
    # ──────────────────────────────────────────────────────
    with open(os.path.join(out_dir, "highway.edg.xml"), "w") as f:
        f.write(f"""<edges>
    <edge id="e0" from="n0" to="n1" numLanes="{road['num_lanes']}" speed="{road['speed_limit']}"/>
    <edge id="e1" from="n1" to="n0" numLanes="{road['num_lanes']}" speed="{road['speed_limit']}"/>
</edges>
""")

    # ──────────────────────────────────────────────────────
    # 3) 라우트 파일: 차량 타입 2종 + 배경 교통 flow
    #
    #    <vType>  : 차량의 물리 속성 템플릿
    #       ego_type : RL이 조종할 차 (빨간색, color="1,0,0" = RGB)
    #    <vTypeDistribution> : 배경차 컨트롤러 믹스.
    #       TRAFFIC["controllers"]에 정의된 여러 차량추종모델(Krauss/IDM/
    #       EIDM/ACC 등)을 probability 비율로 섞는다. flow가 이 분포를
    #       type으로 쓰면, 투입되는 차량마다 SUMO가 확률적으로 타입을 뽑는다.
    #    <route>  : 지나갈 엣지들의 나열. 여기선 e0 하나뿐.
    #    <flow>   : 교통 수요 정의. period="X"는 "X초마다 정확히 1대씩"
    #               → 등간격(uniform) 생성. (vehsPerHour와 등가지만
    #               "비슷한 간격으로 새로 생성"을 명시적으로 보장)
    #               ego는 여기에 없고, 환경(sumo_env.py)이 reset마다
    #               traci.vehicle.add()로 직접 투입한다.
    # ──────────────────────────────────────────────────────
    controllers = traffic.get("controllers")
    if controllers:
        # probability 합이 1이 아니어도 되도록 자동 정규화
        total_p = sum(float(c.get("probability", 1.0))
                      for c in controllers.values())
        vtype_lines = []
        for name, c in controllers.items():
            prob = float(c.get("probability", 1.0)) / total_p
            attrs = [
                f'id="car_{name}"',
                f'probability="{prob:.4f}"',
                f'carFollowModel="{c["carFollowModel"]}"',
                f'accel="{c.get("accel", 2.5)}"',
                f'decel="{c.get("decel", 4.5)}"',
                f'tau="{c.get("tau", 1.0)}"',
                f'minGap="{c.get("min_gap", 2.5)}"',
                # 컨트롤러별 희망속도 오버라이드 (없으면 공통 max_speed).
                # 컨트롤러마다 속도를 다르게 주면 차선 간 속도차가 생겨
                # "추월할 이유"가 자연스럽게 만들어진다.
                f'maxSpeed="{c.get("max_speed", traffic["max_speed"])}"',
                # speedDev: 같은 타입 안에서도 개체별 희망속도 편차 (0~)
                f'speedDev="{c.get("speed_dev", 0.1)}"',
                f'color="{c.get("color", "1,1,0")}"',
                'vClass="passenger"', 'guiShape="passenger/sedan"',
            ]
            # sigma는 Krauss 계열에만 의미가 있으므로 지정된 경우에만 넣는다
            if "sigma" in c:
                attrs.append(f'sigma="{c["sigma"]}"')
            # ── 배경차 차선 유지 (keep_lane, 기본 True) ──
            # 배경차의 자율 차선변경(전략/협조/속도이득/우측유지)을 전부 꺼서
            # 각자 자기 차선을 지키게 한다. 이유:
            #   배경차가 자유롭게 돌아다니면 (측정: 150스텝에 477회 변경)
            #   차선 간 틈이 수시로 닫혀서, ego가 틈을 노려 파고드는
            #   "칼치기 추월"을 학습할 안정적인 기회 자체가 사라진다.
            #   배경차가 차선을 유지하면 "느린 차 뒤 → 옆 틈으로 변경 →
            #   가속"이라는 추월 패턴이 재현 가능한 형태로 나타난다.
            # 특정 컨트롤러만 돌아다니게 하려면 그 항목에 "keep_lane": False.
            if c.get("keep_lane", True):
                attrs += ['lcStrategic="0"', 'lcCooperative="0"',
                          'lcSpeedGain="0"', 'lcKeepRight="0"']
            vtype_lines.append("        <vType " + " ".join(attrs) + "/>")
        vtypes_block = ('    <vTypeDistribution id="carDist">\n'
                        + "\n".join(vtype_lines)
                        + "\n    </vTypeDistribution>")
        flow_type = "carDist"
    else:
        # controllers 설정이 없으면 예전 방식(단일 Krauss)으로 동작 (하위호환)
        vtypes_block = (f'    <vType id="car" accel="2.5" decel="4.5" '
                        f'maxSpeed="{traffic["max_speed"]}" '
                        f'sigma="{traffic.get("sigma", 0.5)}" '
                        f'vClass="passenger" guiShape="passenger/sedan"/>')
        flow_type = "car"

    with open(os.path.join(out_dir, "highway.rou.xml"), "w") as f:
        f.write(f"""<routes>
    <vType id="ego_type" accel="{ego['accel']}" decel="{ego['decel']}" maxSpeed="{ego['max_speed']}" color="1,0,0" vClass="passenger" guiShape="passenger/sedan"/>
{vtypes_block}
    <route id="r0" edges="e0"/>
    <route id="r1" edges="e1"/>
    <flow id="traffic" type="{flow_type}" route="r0" begin="0" end="100000"
          period="{3600.0 / traffic['vehs_per_hour']:.2f}"
          departLane="{traffic['depart_lane']}" departSpeed="{traffic['depart_speed']}"/>
    <flow id="traffic_reverse" type="{flow_type}" route="r1" begin="0" end="100000"
          period="{3600.0 / traffic['vehs_per_hour']:.2f}"
          departLane="{traffic['depart_lane']}" departSpeed="{traffic['depart_speed']}"/>
</routes>
""")

    # ──────────────────────────────────────────────────────
    # 4) GUI 기본 시각화 설정
    #    GUI 관련 값의 source of truth는 env/mdp_config.py의 SIMULATION 하나뿐이다.
    #    view_road.py / train.py / test.py / 데이터 수집 모두 build()를 거치므로
    #    gui_zoom/gui_delay를 바꾸면 생성되는 highway.gui.xml에도 즉시 반영된다.
    # ──────────────────────────────────────────────────────
    # GUI 시작 화면: "도로 시작 구간"이 차선까지 보이게 자동 확대해둔다.
    #   (12km 전체 맞춤으로 시작하면 도로가 가느다란 선으로만 보여 직관성이
    #    떨어지므로, ego가 등장할 시작부를 확대해 대기 화면으로 삼는다.
    #    ▶ 재생 후에는 env가 ego를 추적(trackVehicle)하며 시점을 가져간다.)
    # 좌표/배율은 도로 길이·차선 수에서 "계산"하므로 설정을 바꿔도 안 어긋난다:
    #   zoom(%) = 도로길이 / 보고싶은폭(gui_view_width) × 100
    #     (SUMO zoom은 "네트워크 전체 = 100%" 기준)
    #   y 중심 = -차선폭(3.2m) × 차선수 / 2   (SUMO는 차선을 y<0 쪽으로 그림)
    gui_delay = float(SIMULATION.get("gui_delay", 100.0))
    view_width = float(SIMULATION.get("gui_view_width", 500.0))   # 시작 화면 시야(m)
    zoom = float(road["length"]) / view_width * 100.0
    center_x = view_width * 0.4                                   # 시작점이 화면 왼쪽에 오게
    center_y = -3.2 * int(road["num_lanes"]) / 2.0
    gui_settings_path = os.path.join(out_dir, "highway.gui.xml")
    with open(gui_settings_path, "w") as f:
        f.write(f"""<viewsettings>
    <scheme name="real world"/>
    <viewport zoom="{zoom:.0f}" x="{center_x:.1f}" y="{center_y:.1f}" angle="0"/>
    <delay value="{gui_delay}"/>
</viewsettings>
""")

    # ──────────────────────────────────────────────────────
    # 4.7) 도로변 장식(나무) — 순수 시각 요소
    #
    # SUMO의 <poly>(다각형)로 "초록 수관 + 갈색 줄기"를 그려 도로
    # 양옆에 심는다. additional 파일로 로드되며 차량 물리와 완전히
    # 무관하다 (충돌/관측/보상 어디에도 안 잡힘).
    # 좌표계: 차선은 y ∈ [0, -3.2×차선수] 영역 → 왼쪽 숲은 y > 0,
    # 오른쪽 숲은 y < -3.2×차선수 - side_offset 에 배치.
    # ──────────────────────────────────────────────────────
    _write_scenery(out_dir, road)

    # ──────────────────────────────────────────────────────
    # 5) sumocfg: "이 net 파일과 이 route 파일로 시뮬레이션해라"
    # ──────────────────────────────────────────────────────
    cfg_path = os.path.join(out_dir, "highway.sumocfg")
    with open(cfg_path, "w") as f:
        f.write("""<configuration>
    <input>
        <net-file value="highway.net.xml"/>
        <route-files value="highway.rou.xml"/>
        <additional-files value="highway.scenery.xml"/>
    </input>
    <gui_only>
        <gui-settings-file value="highway.gui.xml"/>
    </gui_only>
</configuration>
""")

    # ──────────────────────────────────────────────────────
    # 6) netconvert 실행: nod + edg → net.xml 컴파일
    #    (SUMO 설치 시 함께 깔리는 커맨드라인 도구)
    # ──────────────────────────────────────────────────────
    netconvert = checkBinary("netconvert")
    subprocess.run(
        [netconvert,
         "--node-files", os.path.join(out_dir, "highway.nod.xml"),
         "--edge-files", os.path.join(out_dir, "highway.edg.xml"),
         "-o", os.path.join(out_dir, "highway.net.xml")],
        check=True,           # 실패하면 예외를 던져서 바로 알 수 있게
        capture_output=True,  # 출력은 조용히 삼킴
    )
    return cfg_path


def _build_sadang_manual(road, out_dir):
    """양재→선암→사당→이수 수작업 모형. 도로 형상과 차로는 설정에서 읽는다."""
    import xml.etree.ElementTree as ET

    def save(root, filename):
        ET.indent(root)
        ET.ElementTree(root).write(os.path.join(out_dir, filename), encoding="utf-8", xml_declaration=True)

    length = float(road["arm_length"])
    geometry = road["ic_geometry"]
    city = road["city_geometry"]
    highway_lanes = int(road["gangnam_lanes"])
    points = {"north": city["north"], "south": tuple(geometry["south_waypoint"]),
              "east": (length, 0), "west": (-length, 0)}
    # 정지선과 차로 연결은 도로의 양 끝점과 중앙 교차로로 정의한다.
    nodes = ET.Element("nodes")
    ET.SubElement(nodes, "node", id="center", x="0", y="0", type="traffic_light")
    for name, (x, y) in points.items():
        ET.SubElement(nodes, "node", id=name, x=str(x), y=str(y))
    reverse_offset = 0  # 반대 방향은 별도 edge로 연속되며 SUMO가 우측에 배치한다.
    wx, wy = geometry["west_end"]
    ex, ey = geometry["east_end"]
    elevated_nodes = {"gyeongbu_south", "gyeongbu_mid", "gyeongbu_north", "gyeongbu_end",
                      "gyeongbu_southbound_start", "gyeongbu_southbound_end",
                      "g_south_mid", "g_south_lower", "g_north_lower",
                      "gangnam_east", "gangnam_east_return"}
    for name, (x, y) in {"isu": city["isu_cross"], "isu_finish": city["isu_finish"],
                         "isu_west": city["isu_west"], "isu_east": city["isu_east"],
                         "south_end": geometry["merge"],
                         "south_road_end": geometry["south_road_end"],
                         "ic_ramp_start": geometry["split"],
                         "gangnam_prepare_start": geometry["exit_prepare"],
                         "gangnam_east": geometry["east_end"],
                         "gangnam_west": geometry["west_end"],
                         "yangjae_start": geometry["yangjae_start"],
                         "heolleung_split": geometry["heolleung_split"],
                         "yangjae_link": geometry["yangjae_link"],
                         "yangjae_fork": geometry["yangjae_fork"],
                         "yangjae_merge": geometry["yangjae_merge"],
                         "yangjae_daero_end": geometry["yangjae_daero_end"],
                         "gyeongbu_south": geometry["gyeongbu_south"],
                         "gyeongbu_mid": geometry["gyeongbu_mid"],
                         "gyeongbu_north": geometry["gyeongbu_north"],
                         "gyeongbu_end": geometry["gyeongbu_end"],
                         "gyeongbu_southbound_start": geometry["gyeongbu_southbound_start"],
                         "gyeongbu_southbound_end": geometry["gyeongbu_southbound_end"],
                         "gangnam_west_return": geometry.get("west_return", (wx, wy+reverse_offset)),
                         "gangnam_east_return": geometry.get("east_return", (ex, ey+reverse_offset))}.items():
        z = geometry["gyeongbu_height"] if name in elevated_nodes else 0
        if name in {"ic_ramp_start","gangnam_prepare_start","gangnam_west","gangnam_west_return"}:
            z=-6
        ET.SubElement(nodes, "node", id=name, x=str(x), y=str(y), z=str(z),
                      type="traffic_light" if name == "isu" else "priority")
    for name, (x,y) in road["yangjae_loop_nodes"].items():
        ET.SubElement(nodes, "node", id=name, x=str(x), y=str(y),
                      z=str(geometry["gyeongbu_height"] if name in elevated_nodes else 0), type="priority")
    for name, (x,y,z) in road["seonam"]["nodes"].items():
        ET.SubElement(nodes, "node", id=name, x=str(x), y=str(y), z=str(z),
                      type=("zipper" if name=="sn_tg_rejoin" else
                            "unregulated" if name.startswith("sn_tg_") else "priority"))
    for node in nodes:
        if node.get("id") in road["yangjae_approach"]["nodes"]:
            for key,value in zip(("x","y","z"),road["yangjae_approach"]["nodes"][node.get("id")]):
                node.set(key,str(value))
        if node.get("id") in road["junction_radii"]:
            node.set("radius",str(road["junction_radii"][node.get("id")]))
    save(nodes, "sadang.nod.xml")

    edges = ET.Element("edges")
    for name in points:
        for suffix, start, end in [("in", name, "center"), ("out", "center", name)]:
            ET.SubElement(edges, "edge", id=f"{name}_{suffix}",
                          **{"from": start, "to": end, "numLanes": str(road["lanes"][name]),
                             "speed": str(road["speed_limit"]), "width": "3.2"})
    # 양방향 도로를 사당IC와 이수역 방향으로 연장한다.
    def shape_string(coords):
        return " ".join(",".join(str(v) for v in point) for point in coords)
    def ramp_3d(points, start_z, end_z):
        # 곡선 길이에 따라 도로 높이를 서서히 바꿔 수직 점프를 막는다.
        import math
        # 표본 좌표의 모서리를 둥글게 하되 분기·합류 끝점은 유지한다.
        for _ in range(2):
            rounded = [points[0]]
            for a, b in zip(points, points[1:]):
                rounded.append((0.75*a[0]+0.25*b[0], 0.75*a[1]+0.25*b[1]))
                rounded.append((0.25*a[0]+0.75*b[0], 0.25*a[1]+0.75*b[1]))
            rounded.append(points[-1])
            points = rounded
        lengths = [math.dist(a,b) for a,b in zip(points, points[1:])]
        total = sum(lengths)
        traveled = 0.0
        output = []
        for i, (x,y) in enumerate(points):
            if i: traveled += lengths[i-1]
            z = start_z + (end_z-start_z)*traveled/total if total else start_z
            output.append((x,y,round(z,2)))
        return output
    for edge_id, start, end, lane_count in [
        ("isu_in", "isu", "north", road["lanes"]["north"]),
        ("isu_out", "north", "isu", road["lanes"]["north"]),
        ("isu_finish_in", "isu_finish", "isu", 4),
        ("isu_finish_out", "isu", "isu_finish", 4),
        ("isu_west_in", "isu_west", "isu", 2), ("isu_west_out", "isu", "isu_west", 2),
        ("isu_east_in", "isu_east", "isu", 2), ("isu_east_out", "isu", "isu_east", 2),
        ("ic_road_in", "south_end", "south", road["lanes"]["south"]),
        ("ic_road_out", "south", "south_end", road["lanes"]["south"]),
        ("ic_upstream_in", "south_road_end", "south_end", road["lanes"]["south"]),
        ("ic_upstream_out", "south_end", "south_road_end", road["lanes"]["south"]),
        ("ic_ramp_in", "ic_ramp_start", "south_end", 1),
    ]:
        attrs = {"from":start, "to":end, "numLanes":str(lane_count),
                 "speed":str(road["speed_limit"]), "width":"3.2"}
        if edge_id == "ic_ramp_in": attrs["shape"] = shape_string(ramp_3d(geometry["ramp_shape"],-6,0))
        if edge_id == "ic_road_in": attrs["shape"] = shape_string(geometry["south_shape"])
        if edge_id == "ic_road_out": attrs["shape"] = shape_string(reversed(geometry["south_shape"]))
        for key, forward, backward in [("isu_shape","isu_out","isu_in"),
                                       ("finish_shape","isu_finish_out","isu_finish_in"),
                                       ("west_shape","isu_west_in","isu_west_out"),
                                       ("east_shape","isu_east_in","isu_east_out")]:
            if edge_id == forward: attrs["shape"] = shape_string(city[key])
            if edge_id == backward: attrs["shape"] = shape_string(reversed(city[key]))
        ET.SubElement(edges, "edge", id=edge_id, **attrs)
    # 양재 사옥 앞 공공도로 → 양재IC 연결부 → 양재대로 → 선암 본선.
    for edge_id, start, end, lanes, shape in [
        ("heolleung_start", "yangjae_start", "heolleung_split", road["heolleung_lanes"], None),
        ("yangjae_ramp", "heolleung_split", "yangjae_link", road["yangjae_link_lanes"], None),
        ("yangjae_daero_a", "yangjae_link", "yangjae_fork", road["yangjae_link_lanes"], None),
        ("yangjae_daero_b", "yangjae_fork", "yangjae_merge", road["yangjae_link_lanes"], None),
        ("yangjae_daero_c", "yangjae_merge", "yangjae_west_fork", road["yangjae_link_lanes"], None),
        ("yangjae_daero_d", "yangjae_west_fork", "sn_prepare", road["yangjae_link_lanes"],
         road["seonam"]["upstream_shape"]),
        ("gyeongbu_in", "gyeongbu_south", "g_north_lower", road["gyeongbu_lanes"], None),
        ("gyeongbu_in_after", "g_north_lower", "gyeongbu_mid", road["gyeongbu_lanes"], None),
        ("gyeongbu_midway", "gyeongbu_mid", "gyeongbu_north", road["gyeongbu_lanes"], None),
        ("gyeongbu_out", "gyeongbu_north", "gyeongbu_end", road["gyeongbu_lanes"], None),
        ("gyeongbu_southbound", "gyeongbu_southbound_start", "g_south_mid", road["gyeongbu_lanes"], None),
        ("gyeongbu_southbound_b", "g_south_mid", "g_south_lower", road["gyeongbu_lanes"], None),
        ("gyeongbu_southbound_c", "g_south_lower", "gyeongbu_southbound_end", road["gyeongbu_lanes"], None),
        ("yj_reverse_a", "yj_reverse_west", "yj_reverse_mid", 2, None),
        ("yj_reverse_b", "yj_reverse_mid", "yj_reverse_fork", 2, None),
        ("yj_reverse_c", "yj_reverse_fork", "yj_reverse_east", 2, None),
        ("loop_nw", "yangjae_west_fork", "g_south_mid", 1, ramp_3d(road["yangjae_loop_shapes"]["loop_nw"],0,geometry["gyeongbu_height"])),
        ("loop_sw", "g_south_lower", "yj_reverse_mid", 1, ramp_3d(road["yangjae_loop_shapes"]["loop_sw"],geometry["gyeongbu_height"],0)),
        ("loop_se", "yj_reverse_fork", "g_north_lower", 1, ramp_3d(road["yangjae_loop_shapes"]["loop_se"],0,geometry["gyeongbu_height"])),
        ("yangjae_to_gyeongbu", "yangjae_fork", "gyeongbu_north", 2, ramp_3d(geometry["yangjae_to_gyeongbu_shape"],0,geometry["gyeongbu_height"])),
        ("gyeongbu_to_yangjae", "gyeongbu_mid", "yangjae_merge", 1, ramp_3d(geometry["gyeongbu_to_yangjae_shape"],geometry["gyeongbu_height"],0)),
    ]:
        attrs = {"from":start, "to":end, "numLanes":str(lanes),
                 "speed":str(road["gangnam_speed"]), "width":"3.2"}
        if shape: attrs["shape"] = shape_string(shape)
        if edge_id=="yj_reverse_c": attrs["shape"]=shape_string(geometry["yangjae_surface_return_shape"])
        ET.SubElement(edges, "edge", id=edge_id, **attrs)
    # 선암IC 지상도로와 지하차도, 선암TG 방향 고가 본선을 별도로 만든다.
    for edge_id, start, end, lanes, speed, shape in road["seonam"]["edges"]:
        attrs = {"from":start, "to":end, "numLanes":str(lanes),
                 "speed":str(speed), "width":"3.2"}
        if shape: attrs["shape"] = shape_string(shape)
        edge = ET.SubElement(edges, "edge", id=edge_id, **attrs)
        if edge_id in road["seonam"]["fixed_lane_edges"]:
            for lane in range(lanes):
                ET.SubElement(edge, "lane", index=str(lane),
                              changeLeft="authority", changeRight="authority")
    # 선암 방면에서 서쪽 사당IC로 접근하는 본선과 그 반대 방향 본선.
    for edge_id, start, end in [("gangnam_in", "gangnam_east", "gangnam_prepare_start"),
                                ("gangnam_exit_prepare", "gangnam_prepare_start", "ic_ramp_start"),
                                ("gangnam_continue", "ic_ramp_start", "gangnam_west"),
                                ("gangnam_return", "gangnam_west_return", "gangnam_east_return")]:
        attrs = {"from":start, "to":end, "numLanes":str(highway_lanes),
                 "speed":str(road["gangnam_speed"]), "width":"3.2"}
        if edge_id == "gangnam_exit_prepare":
            attrs["numLanes"] = str(highway_lanes+1)
            attrs["shape"] = shape_string((x,y,-6) for x,y in [geometry["exit_prepare"],(700,-754),(620,-748),geometry["split"]])
        def highway_height(x):
            # 선암 고가 → 서초터널 → 사당IC 지하 본선. 수치는 층 구분용.
            if x<=800: return -6
            if x<=1250: return round(-6-(x-800)/450*6,2)
            if x<=2307: return -12
            return round(-12+20*min(1,(x-2307)/(geometry["east_return"][0]-2307)),2)
        if edge_id == "gangnam_in":
            attrs["shape"] = shape_string((x,y,highway_height(x)) for x,y in geometry["highway_shape"][:-1])
        if edge_id == "gangnam_return":
            coords = geometry["highway_return_shape"]
            attrs["shape"] = shape_string((x,y,highway_height(x)) for x,y in coords)
        ET.SubElement(edges, "edge", id=edge_id, **attrs)
    for edge in edges:
        if edge.get("id") in ("north_in","north_out"):
            coords = city["north_shape"]
            edge.set("shape", shape_string(coords if edge.get("id")=="north_out" else reversed(coords)))
        if edge.get("id") in road["edge_end_offsets"]:
            edge.set("endOffset", str(road["edge_end_offsets"][edge.get("id")]))
        if edge.get("id") in road["city_bus_lanes"]:
            ET.SubElement(edge, "lane", index=str(road["city_bus_lanes"][edge.get("id")]), allow="bus")
    # 직각 진입부를 제거하고 지하차도 진입 직선으로 교체한다.
    for edge in edges:
        eid=edge.get("id")
        if eid in road["yangjae_approach"]["shapes"]:
            edge.set("shape",shape_string(road["yangjae_approach"]["shapes"][eid]))
        if eid=="heolleung_return": edge.set("numLanes","2")
    # 양재 중앙 통과도로 옆의 연결도로를 같은 선형에 맞춘다.
    # 노드 ID와 connection은 그대로 두어 두 도로 사이 횡단 이동을 막는다.
    import math
    from sumolib.geomhelper import move2side
    node_map = {n.get("id"): n for n in nodes}
    edge_map = {e.get("id"): e for e in edges}
    def edge_points(e):
        if e.get("shape"):
            return [tuple(map(float, p.split(","))) for p in e.get("shape").split()]
        return [tuple(float(node_map[e.get(k)].get(c, "0")) for c in ("x","y","z"))
                for k in ("from", "to")]
    distance = (2*road["lane_widths"]["motorway"] +
                2*road["lane_widths"]["arterial"])/2 + road["yangjae_parallel_gap"]
    corridors = [
        (edge_points(edge_map["seonam_main_in"]),
         ["yangjae_daero_a","yangjae_daero_b","yangjae_daero_c","yangjae_daero_d"]),
        (edge_points(edge_map["yangjae_central_return"]),
         ["yj_reverse_a","yj_reverse_b","yj_reverse_c"]),
    ]
    moved_nodes = {}
    for axis, ids in corridors:
        def align(p):
            # 정렬 구간 양끝에서는 기존 선형에 부드럽게 복귀한다.
            weight = min(1., max(0., (p[0]-4860)/70), max(0., (5390-p[0])/70))
            if not weight: return p
            nearest = None
            for a,b in zip(axis,axis[1:]):
                dx,dy = b[0]-a[0],b[1]-a[1]
                length = math.hypot(dx,dy)
                if not length: continue
                t = max(0.,min(1.,((p[0]-a[0])*dx+(p[1]-a[1])*dy)/length**2))
                q = (a[0]+t*dx,a[1]+t*dy)
                candidate = (math.dist(p[:2],q), q[0]+distance*dy/length, q[1]-distance*dx/length)
                if nearest is None or candidate[0]<nearest[0]: nearest=candidate
            return (p[0]*(1-weight)+nearest[1]*weight,
                    p[1]*(1-weight)+nearest[2]*weight,*p[2:])
        for eid in ids:
            edge = edge_map[eid]
            ps = [align(p) for p in edge_points(edge)]
            edge.set("shape",shape_string(ps))
            moved_nodes[edge.get("from")] = ps[0]
            moved_nodes[edge.get("to")] = ps[-1]
    for nid,p in moved_nodes.items():
        for key,value in zip(("x","y"),p): node_map[nid].set(key,str(value))
    # 램프도 옮겨진 분기점에 접속하도록 끝점을 함께 맞춘다.
    for edge in edges:
        if edge.get("shape"):
            ps=edge_points(edge)
            for key,index in (("from",0),("to",-1)):
                if edge.get(key) in moved_nodes:
                    p=moved_nodes[edge.get(key)]
                    ps[index]=(p[0],p[1],*ps[index][2:])
            edge.set("shape",shape_string(ps))
    save(nodes, "sadang.nod.xml")
    # 지도에서 대조한 방향별 중앙선은 centered spread를 사용한다.
    # 양방향이 하나의 기준선을 공유하는 도시 도로만 right spread로 나눈다.
    import math
    from sumolib.geomhelper import move2side
    shared_axis={f"{arm}_{direction}" for arm in points for direction in ("in","out")}
    shared_axis.update({"isu_in","isu_out","isu_finish_in","isu_finish_out",
                        "isu_west_in","isu_west_out","isu_east_in","isu_east_out",
                        "ic_road_in","ic_road_out","ic_upstream_in","ic_upstream_out"})
    def rounded_shape(ps):
        if len(ps)<3 or len(ps)>40: return ps
        result=[ps[0]]
        for a,b,c in zip(ps,ps[1:],ps[2:]):
            incoming,outgoing=math.dist(a[:2],b[:2]),math.dist(b[:2],c[:2])
            if min(incoming,outgoing)<1:
                result.append(b); continue
            u,v=(b[0]-a[0],b[1]-a[1]),(c[0]-b[0],c[1]-b[1])
            cosine=(u[0]*v[0]+u[1]*v[1])/(incoming*outgoing)
            if cosine<math.cos(math.radians(80)):
                result.append(b); continue  # 잘못된 급반전은 아래 검증에서 숨기지 않는다.
            cut=min(road["curve_rounding"],incoming*.2,outgoing*.2)
            p=tuple(y+(x-y)*cut/incoming for x,y in zip(a,b))
            q=tuple(y+(x-y)*cut/outgoing for x,y in zip(c,b))
            for t in (0,.25,.5,.75,1):
                result.append(tuple(round((1-t)**2*x+2*t*(1-t)*y+t*t*z,3)
                                    for x,y,z in zip(p,b,q)))
        result.append(ps[-1])
        return result
    for edge in edges:
        eid=edge.get("id")
        motorway=eid.startswith(("gyeongbu_","gangnam_","seonam_main","seonam_return","seonam_after"))
        kind="toll" if eid.startswith("seonam_tg_") else "ramp" if int(edge.get("numLanes"))==1 else "motorway" if motorway else "arterial"
        width=road["lane_widths"][kind]
        edge.set("width",str(width))
        edge.set("spreadType","right" if eid in shared_axis else "center")
        if edge.get("shape"):
            ps=[tuple(map(float,p.split(","))) for p in edge.get("shape").split()]
            # 오른쪽 출구 차로 추가 / 왼쪽 고가 진입 차로 추가 시
            # 기존 차로의 횡방향 위치를 유지하도록 중앙선을 반 차로 옮긴다.
            shift={"gangnam_exit_prepare":width/2,"seonam_lane_prepare":-width/2}.get(eid,0)
            if shift:
                moved=move2side([p[:2] for p in ps],shift)
                ps=[(*xy,*p[2:]) for xy,p in zip(moved,ps)]
            edge.set("shape",shape_string(rounded_shape(ps)))
    # 도로 분할 후 과거 끝점을 남겨 두는 실수를 생성 단계에서 차단한다.
    import math
    for edge in edges:
        if not edge.get("shape"):
            continue
        ps=[tuple(map(float,p.split(",")[:2])) for p in edge.get("shape").split()]
        for a,b,c in zip(ps,ps[1:],ps[2:]):
            u,v=(b[0]-a[0],b[1]-a[1]),(c[0]-b[0],c[1]-b[1])
            scale=math.hypot(*u)*math.hypot(*v)
            if scale and (u[0]*v[0]+u[1]*v[1])/scale < math.cos(math.radians(110)):
                raise ValueError(f"도로 {edge.get('id')}의 선형이 되꺾입니다. 분할 끝점과 좌표를 확인하세요.")
    save(edges, "sadang.edg.xml")

    # SUMO 차로 번호는 오른쪽부터 0, 1, 2... 이다.
    opposite = {"north":"south", "south":"north", "east":"west", "west":"east"}
    right = {"north":"west", "south":"east", "east":"north", "west":"south"}
    left = {"north":"east", "south":"west", "east":"south", "west":"north"}
    connections = ET.Element("connections")
    def connect(a, b, from_lane, to_lane):
        ET.SubElement(connections, "connection", **{"from": f"{a}_in", "to": f"{b}_out",
                          "fromLane": str(from_lane), "toLane": str(to_lane)})
    for a in points:
        n = int(road["lanes"][a])
        b = opposite[a]
        for lane in range(min(n, int(road["lanes"][b]))):
            connect(a, b, lane, lane)
        connect(a, right[a], 0, 0)  # 임시: 오른쪽 끝 차로에서 우회전
        dest = left[a]
        dest_lane = int(road["lanes"][dest])-1
        if f"{dest}_out" in road["city_bus_lanes"]:
            dest_lane -= 1  # 일반 차량의 좌회전이 버스전용차로에 연결되지 않도록 한다.
        from_lane = n-1
        if f"{a}_in" in road["city_bus_lanes"]:
            from_lane -= 1
        connect(a, dest, from_lane, dest_lane)
    # 긴 도로는 각 중간 지점에서도 같은 번호의 차로로 이어진다.
    for prefix, count in [("north", road["lanes"]["north"]), ("south", road["lanes"]["south"])]:
        before, after = ("isu_in", "isu_out") if prefix == "north" else ("ic_road_in", "ic_road_out")
        for lane in range(int(count)):
            for a, b in [(before, f"{prefix}_in"), (f"{prefix}_out", after)]:
                ET.SubElement(connections, "connection", **{"from":a, "to":b,
                              "fromLane":str(lane), "toLane":str(lane)})
    ET.SubElement(connections, "connection", **{"from":"ic_ramp_in", "to":"ic_road_in",
                  "fromLane":"0", "toLane":"0"})
    for lane in range(int(road["lanes"]["south"])):
        for a,b in [("ic_upstream_in","ic_road_in"), ("ic_road_out","ic_upstream_out")]:
            ET.SubElement(connections, "connection", **{"from":a, "to":b,
                          "fromLane":str(lane), "toLane":str(lane)})
    # 맨 오른쪽 차로(0)에서 진출하고, 다른 차량은 본선으로 계속 간다.
    for lane in range(highway_lanes):
        for a,b,from_lane,to_lane in [("gangnam_in", "gangnam_exit_prepare",lane,lane+1),
                                      ("gangnam_exit_prepare", "gangnam_continue",lane+1,lane)]:
            ET.SubElement(connections, "connection", **{"from":a, "to":b,
                          "fromLane":str(from_lane), "toLane":str(to_lane)})
    ET.SubElement(connections, "connection", **{"from":"gangnam_exit_prepare", "to":"ic_ramp_in",
                  "fromLane":"0", "toLane":"0"})
    # 해당 경로로 갈 수 있는 차로 번호는 임시값이며 현장 대조가 필요하다.
    for a,b,count in [("heolleung_start", "yangjae_ramp", min(road["heolleung_lanes"],road["yangjae_link_lanes"])),
                      ("yangjae_ramp", "yangjae_daero_a", road["yangjae_link_lanes"]),
                      ("yangjae_daero_a", "yangjae_daero_b", road["yangjae_link_lanes"]),
                      ("yangjae_daero_b", "yangjae_daero_c", road["yangjae_link_lanes"]),
                      ("yangjae_daero_c", "yangjae_daero_d", road["yangjae_link_lanes"]),
                      ("yangjae_daero_d", "seonam_lane_prepare", 2),
                      ("seonam_after_tg", "gangnam_in", highway_lanes),
                      ("gangnam_return", "seonam_return", highway_lanes),
                      ("seonam_surface_w", "seonam_surface_ramp", 2),
                      ("seonam_surface_ramp", "seonam_underpass_w", 2),
                      ("seonam_underpass_e", "seonam_surface_e", 2),
                      ("seonam_surface_e", "seonam_surface_e2", 2),
                      ("gyeongbu_in", "gyeongbu_in_after", road["gyeongbu_lanes"]),
                      ("gyeongbu_in_after", "gyeongbu_midway", road["gyeongbu_lanes"]),
                      ("gyeongbu_southbound", "gyeongbu_southbound_b", road["gyeongbu_lanes"]),
                      ("gyeongbu_southbound_b", "gyeongbu_southbound_c", road["gyeongbu_lanes"]),
                      ("yj_reverse_a", "yj_reverse_b", 2), ("yj_reverse_b", "yj_reverse_c", 2),
                      ("gyeongbu_midway", "gyeongbu_out", road["gyeongbu_lanes"])]:
        for lane in range(int(count)):
            ET.SubElement(connections, "connection", **{"from":a, "to":b,
                          "fromLane":str(lane), "toLane":str(lane)})
    for a,b,from_lane,to_lane in [("yangjae_daero_a","yangjae_to_gyeongbu",0,0),
                                   ("yangjae_daero_a","yangjae_to_gyeongbu",1,1),
                                   ("yangjae_to_gyeongbu","gyeongbu_out",0,0),
                                   ("yangjae_to_gyeongbu","gyeongbu_out",1,1),
                                   ("gyeongbu_in_after","gyeongbu_to_yangjae",0,0),
                                   ("gyeongbu_to_yangjae","yangjae_daero_c",0,0),
                                   ("yangjae_daero_c","loop_nw",0,0),
                                   ("loop_nw","gyeongbu_southbound_b",0,0),
                                   ("gyeongbu_southbound_b","loop_sw",0,0),
                                   ("loop_sw","yj_reverse_b",0,0),
                                   ("yj_reverse_b","loop_se",0,0),
                                   ("loop_se","gyeongbu_in_after",0,0)]:
        ET.SubElement(connections, "connection", **{"from":a, "to":b,
                      "fromLane":str(from_lane), "toLane":str(to_lane)})
    for a,b,from_lane,to_lane in road["seonam"]["connections"]:
        ET.SubElement(connections, "connection", **{"from":a, "to":b,
                      "fromLane":str(from_lane), "toLane":str(to_lane)})
    for lane in range(6):
        for a,b,from_lane,to_lane in [
                ("seonam_main","seonam_tg_fan",lane//2,lane),
                ("seonam_tg_fan","seonam_tg_gate",lane,lane),
                ("seonam_tg_gate","seonam_tg_rejoin",lane,lane),
                ("seonam_tg_rejoin","seonam_after_tg",lane,lane//2)]:
            ET.SubElement(connections, "connection", **{"from":a,"to":b,
                          "fromLane":str(from_lane),"toLane":str(to_lane)})
    # 이수 사거리: 남북 본선, 사당로/서초대로를 실제 위치에서 접속한다.
    for lane in range(4):
        for a,b in [("isu_out","isu_finish_out"),("isu_finish_in","isu_in")]:
            ET.SubElement(connections,"connection",**{"from":a,"to":b,"fromLane":str(lane),"toLane":str(lane)})
    for lane in range(2):
        for a,b in [("isu_west_in","isu_east_out"),("isu_east_in","isu_west_out")]:
            ET.SubElement(connections,"connection",**{"from":a,"to":b,"fromLane":str(lane),"toLane":str(lane)})
    for a,b,i,j in [("isu_out","isu_east_out",0,0),("isu_out","isu_west_out",2,1),
                     ("isu_finish_in","isu_west_out",0,0),("isu_finish_in","isu_east_out",2,1),
                     ("isu_west_in","isu_in",0,0),("isu_west_in","isu_finish_out",1,2),
                     ("isu_east_in","isu_finish_out",0,0),("isu_east_in","isu_in",1,2)]:
        ET.SubElement(connections,"connection",**{"from":a,"to":b,"fromLane":str(i),"toLane":str(j)})
    # 매헌로 구간: 지하차도 출구 + IC 측도를 하나의 왕복 8차로로 합친다.
    # 이후의 삼거리는 두 본선 방향과 짧은 옆길 하나만 갖는다.
    mh=road["maeheon_corridor"]
    for nid,point,kind in [("mh_east",mh["east"],"priority"),
                           ("mh_t",mh["junction"],"traffic_light"),
                           ("mh_side_end",mh["side_end"],"dead_end")]:
        ET.SubElement(nodes,"node",id=nid,x=str(point[0]),y=str(point[1]),z=str(point[2]),type=kind)
    em={e.get("id"):e for e in edges}
    nm={n.get("id"):n for n in nodes}
    def node_point(nid): return tuple(float(nm[nid].get(c,"0")) for c in ("x","y","z"))
    def reshape(eid,start,end,ps,lanes=None):
        e=em[eid]
        e.set("from",start); e.set("to",end)
        e.set("shape",shape_string(rounded_shape(ps)))
        if lanes is not None: e.set("numLanes",str(lanes))
    # 네 개의 2차로 끝을 같은 횡단면의 차로 위치에 맞춘다.
    # 같은 점으로 모았다 벌리는 방식은 긴 대각선과 큰 접속면을 만든다.
    vx,vy=mh["junction"][0]-mh["east"][0],mh["junction"][1]-mh["east"][1]
    norm=math.hypot(vx,vy); ux,uy=vx/norm,vy/norm
    rx,ry=uy,-ux
    lane_width=road["lane_widths"]["arterial"]
    carriageway_shift=(mh["after_t_lanes"]-mh["lanes"])*lane_width if mh["align_outer_edges"] else 0
    carriageway_shift*=mh["median_scale"]
    def port(offset):
        return (mh["east"][0]+rx*offset,mh["east"][1]+ry*offset,0)
    wb_outer,wb_inner=port(carriageway_shift+3*lane_width),port(carriageway_shift+lane_width)
    eb_outer,eb_inner=port(-carriageway_shift-3*lane_width),port(-carriageway_shift-lane_width)
    reshape("yangjae_daero_d","yangjae_west_fork","mh_east",
            [node_point("yangjae_west_fork"),wb_outer],2)
    em["yangjae_daero_d"].attrib.pop("endOffset",None)
    approach=road["yangjae_approach"]["shapes"]["seonam_main_in"][:6]
    reshape("seonam_main_in","sn_main_east","mh_east",approach+
            [(5035,-1673,-2),wb_inner])
    em["seonam_main_in"].set("width",str(lane_width))
    reshape("seonam_local_to_yangjae","sn_local_east_end","sn_prepare",
            [node_point("sn_local_east_end"),(4585,-2006,0),node_point("sn_prepare")])
    reshape("seonam_return_east","sn_return_merge","sn_prepare",
            [node_point("sn_return_merge"),(4466,-2007,2),(4535,-2000,0),node_point("sn_prepare")])
    reshape("yj_reverse_a","mh_east","yj_reverse_mid",
            [eb_outer,node_point("yj_reverse_mid")])
    returning=road["yangjae_approach"]["shapes"]["yangjae_central_return"]
    reshape("yangjae_central_return","mh_east","sn_return_end",
            [eb_inner,(5038,-1686,-2)]+returning[1:])
    em["yangjae_central_return"].set("width",str(lane_width))
    # 짧은 실제 접속 노면. 양쪽 진행 방향은 중앙선 양편에서 평행하게 이어진다.
    junction_outline=[(mh["east"][0]+along*ux+across*rx,
                       mh["east"][1]+along*uy+across*ry)
                      for along,across in [(-2,-carriageway_shift-4*lane_width),(-2,carriageway_shift+4*lane_width),
                                            (2,carriageway_shift+4*lane_width),(2,-carriageway_shift-4*lane_width)]]
    nm["mh_east"].set("shape",shape_string(junction_outline))
    nm["mh_east"].set("radius","0")
    em["seonam_lane_prepare"].set("numLanes","4")
    main_sections=[("mh_wb_e","mh_east","mh_t",[mh["east"],mh["junction"]]),
                   ("mh_wb_w","mh_t","sn_prepare",mh["west_shape"]),
                   ("mh_eb_w","sn_prepare","mh_t",list(reversed(mh["west_shape"]))),
                   ("mh_eb_e","mh_t","mh_east",[mh["junction"],mh["east"]])]
    for eid,start,end,ps in main_sections:
        if eid in ("mh_wb_e","mh_eb_e") and carriageway_shift:
            # 7차로 바깥 경계를 기준으로 양 방향 4차로를 벌린다(간격 배율 적용).
            # 삼거리 직전 접선도 다음 구간과 같게 하여 꺾임을 줄인다.
            tx,ty=mh["crossroad"][0]-mh["junction"][0],mh["crossroad"][1]-mh["junction"][1]
            distance=math.hypot(tx,ty)
            anchor=(mh["junction"][0]-tx/distance*45,mh["junction"][1]-ty/distance*45,0)
            axis=[mh["east"],anchor,mh["junction"]]
            if eid=="mh_eb_e": axis=list(reversed(axis))
            ps=[(*p,0) for p in move2side([p[:2] for p in rounded_shape(axis)],carriageway_shift)]
        ET.SubElement(edges,"edge",id=eid,**{"from":start,"to":end,
                      "numLanes":str(mh["lanes"]),"width":str(road["lane_widths"]["arterial"]),
                      "speed":"16.67","spreadType":"right","shape":shape_string(rounded_shape(ps))})
    for eid,start,end,ps in [("mh_side_in","mh_side_end","mh_t",[mh["side_end"],mh["junction"]]),
                             ("mh_side_out","mh_t","mh_side_end",[mh["junction"],mh["side_end"]])]:
        ET.SubElement(edges,"edge",id=eid,**{"from":start,"to":end,"numLanes":str(mh["side_lanes"]),
                      "width":"3.25","speed":"11.11","spreadType":"right","shape":shape_string(ps)})
    # 기존의 겹치는 평행도로 연결을 제거하고 명시적인 합류/분기만 허용한다.
    removed_pairs={("yangjae_daero_d","seonam_lane_prepare"),
                   ("seonam_main_in","seonam_main"),
                   ("seonam_return_east","yangjae_central_return"),
                   ("seonam_local_to_yangjae","yj_reverse_a")}
    for c in list(connections):
        pair=(c.get("from"),c.get("to"))
        if pair in removed_pairs: connections.remove(c)
        elif pair==("seonam_lane_prepare","seonam_approach"): c.set("fromLane","3")
    def connect(a,b,i,j):
        ET.SubElement(connections,"connection",**{"from":a,"to":b,"fromLane":str(i),"toLane":str(j)})
    for i in range(4):
        connect("mh_wb_e","mh_wb_w",i,i)
        connect("mh_wb_w","seonam_lane_prepare",i,i)
        connect("mh_eb_w","mh_eb_e",i,i)
    for i in range(2):
        connect("yangjae_daero_d","mh_wb_e",i,i)
        connect("seonam_main_in","mh_wb_e",i,i+2)
        connect("seonam_local_to_yangjae","mh_eb_w",i,i)
        connect("seonam_return_east","mh_eb_w",i,i+2)
        connect("mh_eb_e","yj_reverse_a",i,i)
        connect("mh_eb_e","yangjae_central_return",i+2,i)
    connect("seonam_return_east","mh_eb_w",2,4)
    connect("seonam_lane_prepare","seonam_surface_w",2,1)
    for a,b,i,j in [("mh_wb_e","mh_side_out",0,0),("mh_eb_w","mh_side_out",3,0),
                     ("mh_side_in","mh_wb_w",0,0),("mh_side_in","mh_eb_e",0,3)]:
        connect(a,b,i,j)
    # 삼거리 이후 편도 7차로, 다음 곡선부는 신호 사거리.
    wide=mh["after_t_lanes"]
    for nid,point,kind in [("mh_cross",mh["crossroad"],"traffic_light"),
                           ("mh_cross_n",mh["cross_north_end"],"dead_end"),
                           ("mh_cross_s",mh["cross_south_end"],"dead_end")]:
        ET.SubElement(nodes,"node",id=nid,x=str(point[0]),y=str(point[1]),z="0",type=kind)
    section_map={e.get("id"):e for e in edges}
    for eid,start,end,ps in [
            ("mh_wb_w","mh_t","mh_cross",[mh["junction"],mh["crossroad"]]),
            ("mh_eb_w","mh_cross","mh_t",[mh["crossroad"],mh["junction"]])]:
        e=section_map[eid]
        e.set("from",start);e.set("to",end);e.set("numLanes",str(wide))
        e.set("shape",shape_string(ps))
    for eid,start,end,ps in [
            ("mh_wb_after","mh_cross","sn_prepare",[mh["crossroad"],(4662,-1961,0),node_point("sn_prepare")]),
            ("mh_eb_before","sn_prepare","mh_cross",[node_point("sn_prepare"),(4662,-1961,0),mh["crossroad"]])]:
        ET.SubElement(edges,"edge",id=eid,**{"from":start,"to":end,"numLanes":str(wide),
                      "width":str(lane_width),"speed":"16.67","spreadType":"right",
                      "shape":shape_string(rounded_shape(ps))})
    for direction in ("n","s"):
        point=mh["cross_north_end" if direction=="n" else "cross_south_end"]
        for flow,start,end,ps in [("in","mh_cross_"+direction,"mh_cross",[point,mh["crossroad"]]),
                                  ("out","mh_cross","mh_cross_"+direction,[mh["crossroad"],point])]:
            ET.SubElement(edges,"edge",id="mh_cross_"+direction+"_"+flow,
                          **{"from":start,"to":end,"numLanes":str(mh["cross_side_lanes"]),
                             "width":str(lane_width),"speed":"11.11","spreadType":"right","shape":shape_string(ps)})
    # 기존 삼거리/서쪽 연결을 새 사거리 앞뒤 구간에 이식한다.
    for c in connections:
        if c.get("from")=="mh_wb_w": c.set("from","mh_wb_after")
        if c.get("to")=="mh_eb_w": c.set("to","mh_eb_before")
        if (c.get("from"),c.get("to"))==("mh_eb_w","mh_side_out"):
            c.set("fromLane",str(wide-1))
    for c in list(connections):
        if (c.get("from"),c.get("to")) in {
                ("mh_wb_e","mh_wb_w"),("mh_eb_w","mh_eb_e"),
                ("mh_wb_after","seonam_lane_prepare")}:
            connections.remove(c)
    for i in range(wide):
        # 바깥 4차로는 같은 차로 번호로 직진하고 새 3차로는 중앙 쪽에 생긴다.
        connect("mh_wb_e","mh_wb_w",min(i,3),i)
        connect("mh_eb_w","mh_eb_e",i,min(i,3))
        connect("mh_wb_w","mh_wb_after",i,i)
        connect("mh_eb_before","mh_eb_w",i,i)
        connect("mh_wb_after","seonam_lane_prepare",i,min(3,i//2))
    # 본선/횡단도로 각 접근로에서 직진, 좌회전, 우회전이 가능하다.
    for a,b,i,j in [
            ("mh_wb_w","mh_cross_n_out",0,0),("mh_wb_w","mh_cross_s_out",wide-1,1),
            ("mh_eb_before","mh_cross_s_out",0,0),("mh_eb_before","mh_cross_n_out",wide-1,1),
            ("mh_cross_n_in","mh_wb_after",0,0),("mh_cross_n_in","mh_eb_w",1,wide-1),
            ("mh_cross_s_in","mh_eb_w",0,0),("mh_cross_s_in","mh_wb_after",1,wide-1)]:
        connect(a,b,i,j)
    for i in range(mh["cross_side_lanes"]):
        connect("mh_cross_n_in","mh_cross_s_out",i,i)
        connect("mh_cross_s_in","mh_cross_n_out",i,i)
    # 선암 분기: 왼쪽부터 고가 본선2 / 과천1 / 고가 진입1 / 지상3.
    split=road["seonam_split"]
    all_edges={e.get("id"):e for e in edges}
    prepare=all_edges["seonam_lane_prepare"]
    prepare.set("numLanes","7");prepare.set("spreadType","right")
    prepare_points=next(ps for eid,_,_,_,_,ps in road["seonam"]["edges"] if eid=="seonam_lane_prepare")
    # 7차로 곡선의 끝과 다음 7차로 시작을 같은 접선으로 잇는다.
    # 반대 방향 3+2차로도 가운데 점이 아니라 해당 차로의 횡단면에 붙인다.
    def cubic(a,b,c,d):
        return [tuple((1-t)**3*a[k]+3*(1-t)**2*t*b[k]+3*(1-t)*t*t*c[k]+t**3*d[k]
                      for k in range(3)) for t in (i/12 for i in range(13))]
    join=node_point("sn_prepare")
    previous=mh["west_shape"][-2]
    jx,jy=previous[0]-join[0],previous[1]-join[1]
    norm=math.hypot(jx,jy);jx,jy=jx/norm,jy/norm
    jr=(jy,-jx)
    end=prepare_points[-1]
    ex,ey=end[0]-prepare_points[-2][0],end[1]-prepare_points[-2][1]
    norm=math.hypot(ex,ey);ex,ey=ex/norm,ey/norm
    prepare_points=cubic(join,(join[0]-jx*65,join[1]-jy*65,0),
                         (end[0]-ex*80,end[1]-ey*80,0),end)
    local_port=(join[0]+jr[0]*6*lane_width,join[1]+jr[1]*6*lane_width,0)
    return_port=(join[0]+jr[0]*3.5*lane_width,join[1]+jr[1]*3.5*lane_width,0)
    local_start=node_point("sn_local_east_end")
    reshape("seonam_local_to_yangjae","sn_local_east_end","sn_prepare",
            cubic(local_start,(local_start[0]+28,local_start[1]-3,0),
                  (local_port[0]-jx*45,local_port[1]-jy*45,0),local_port))
    return_bend=(4466,-2007,2)
    reshape("seonam_return_east","sn_return_merge","sn_prepare",
            [node_point("sn_return_merge")]+cubic(return_bend,(4500,-2007,1),
                   (return_port[0]-jx*60,return_port[1]-jy*60,0),return_port))
    all_edges["seonam_return_east"].set("width",str(lane_width))
    join_outline=[(join[0]+along*jx+across*jr[0],join[1]+along*jy+across*jr[1])
                  for along,across in [(-2,-7*lane_width),(-2,7*lane_width),
                                        (2,7*lane_width),(2,-7*lane_width)]]
    nm["sn_prepare"].set("shape",shape_string(join_outline))
    nm["sn_prepare"].set("radius","0")
    prepare.set("shape",shape_string(rounded_shape(prepare_points)))
    split_origin=node_point("yangjae_daero_end")
    ax,ay=prepare_points[-1][0]-prepare_points[-2][0],prepare_points[-1][1]-prepare_points[-2][1]
    norm=math.hypot(ax,ay);sx,sy=ay/norm,-ax/norm
    def split_port(indices):
        offset=(7-(sum(indices)/len(indices)+.5))*lane_width
        return (split_origin[0]+sx*offset,split_origin[1]+sy*offset,0)
    merge=node_point("sn_merge")
    mx,my=-48.,5.;length=math.hypot(mx,my);mr=(my/length,-mx/length)
    main_width=road["lane_widths"]["motorway"]
    toll_main_end=(merge[0]-mr[0]*main_width/2,merge[1]-mr[1]*main_width/2,8)
    toll_entry_end=(merge[0]+mr[0]*main_width,merge[1]+mr[1]*main_width,8)
    reshape("seonam_approach","yangjae_daero_end","sn_merge",
            [split_port(split["toll_entry_lanes"])]+split["toll_entry_shape"]+[toll_entry_end],1)
    reshape("seonam_surface_w","yangjae_daero_end","sn_local_w",
            [split_port(split["surface_lanes"])]+split["surface_shape"]+[node_point("sn_local_w")],3)
    ET.SubElement(nodes,"node",id="sn_banpo_end",x=str(split["banpo_end"][0]),
                  y=str(split["banpo_end"][1]),z="0",type="dead_end")
    for eid,end,lanes,ps in [
            ("seonam_tg_direct","sn_merge",2,[split_port(split["toll_main_lanes"])]+split["toll_main_shape"]+[toll_main_end]),
            ("seonam_gwacheon","sn_local_ramp",1,[split_port(split["gwacheon_lanes"])]+split["gwacheon_shape"]+[node_point("sn_local_ramp")])]:
        ET.SubElement(edges,"edge",id=eid,**{"from":"yangjae_daero_end","to":end,
                      "numLanes":str(lanes),"width":str(lane_width),"speed":"16.67",
                      "spreadType":"center","shape":shape_string(rounded_shape(ps))})
    ET.SubElement(edges,"edge",id="seonam_banpo_ground",**{"from":"sn_local_w","to":"sn_banpo_end",
                  "numLanes":"3","width":str(lane_width),"speed":"16.67","spreadType":"center",
                  "shape":shape_string(rounded_shape([node_point("sn_local_w")]+split["banpo_shape"]+[split["banpo_end"]]))})
    # 지상 3차로는 고도를 0으로 유지한다. 고가 2+1은 위에서 3차로로 합류한다.
    for c in list(connections):
        if c.get("from")=="seonam_lane_prepare" or (c.get("from"),c.get("to"))==("seonam_approach","seonam_main"):
            connections.remove(c)
        elif (c.get("from"),c.get("to"))==("mh_wb_after","seonam_lane_prepare"):
            c.set("toLane",c.get("fromLane"))
    for key,target in [("surface_lanes","seonam_surface_w"),("toll_entry_lanes","seonam_approach"),
                       ("gwacheon_lanes","seonam_gwacheon"),("toll_main_lanes","seonam_tg_direct")]:
        for destination,source in enumerate(split[key]): connect("seonam_lane_prepare",target,source,destination)
    connect("seonam_approach","seonam_main",0,0)
    for i in range(2): connect("seonam_tg_direct","seonam_main",i,i+1)
    connect("seonam_gwacheon","umyeon_entry",0,0)
    for i in range(3): connect("seonam_surface_w","seonam_banpo_ground",i,i)
    # 서쪽의 불필요한 교차도로 묶음만 제거한다. 네 갈래는 경계에서 종료한다.
    removed_edges=set(road["seonam_cleanup"]["remove_edges"])
    boundary=road["seonam_cleanup"]["gwacheon_end"]
    ET.SubElement(nodes,"node",id="sn_gwacheon_end",x=str(boundary[0]),y=str(boundary[1]),z=str(boundary[2]),type="dead_end")
    for e in list(edges):
        if e.get("id") in removed_edges:
            edges.remove(e)
        elif e.get("id")=="seonam_gwacheon":
            ps=[tuple(map(float,p.split(","))) for p in e.get("shape").split()]
            ps=[p for p in ps if p[0]>boundary[0]]+[boundary]
            e.set("to","sn_gwacheon_end");e.set("shape",shape_string(ps))
    for c in list(connections):
        if c.get("from") in removed_edges or c.get("to") in removed_edges:
            connections.remove(c)
        elif (c.get("from"),c.get("to")) in {
                ("seonam_return","seonam_return_main"),
                ("seonam_return_main","seonam_return_east")}:
            connections.remove(c)
    # 반대편 본선은 중간에 3→2→3차로로 끊지 않고 3차로로 연속 연결한다.
    for e in edges:
        if e.get("id") in {"seonam_return","seonam_return_main","seonam_return_east"}:
            e.set("numLanes","3");e.set("width",str(lane_width))
    for i in range(3):
        connect("seonam_return","seonam_return_main",i,i)
        connect("seonam_return_main","seonam_return_east",i,i)
    for n in nodes:
        if n.get("id") in {"sn_return_split","sn_return_merge"}:
            n.set("type","unregulated")
    # 남은 선암 병행 도로는 하나의 완만한 기준선에서 횡방향으로 배치한다.
    # 차로 수와 분기 순서는 유지하고, 필요한 진출입 램프만 옆 도로로 이동한다.
    layout=road["seonam_layout"]
    anchors=layout["axis"]
    slopes=[]
    for i,(x,y) in enumerate(anchors):
        a,b=anchors[max(0,i-1)],anchors[min(len(anchors)-1,i+1)]
        slopes.append((b[1]-a[1])/(b[0]-a[0]))
    slopes[-1]=jy/jx  # 앞 7차로 곡선의 접선을 이어받는다.
    def axis_y(x):
        for i,(a,b) in enumerate(zip(anchors,anchors[1:])):
            if a[0]<=x<=b[0]:
                h=b[0]-a[0];t=(x-a[0])/h
                return ((2*t**3-3*t*t+1)*a[1]+(t**3-2*t*t+t)*h*slopes[i]
                        +(-2*t**3+3*t*t)*b[1]+(t**3-t*t)*h*slopes[i+1])
        p=anchors[0] if x<anchors[0][0] else anchors[-1]
        m=slopes[0] if x<anchors[0][0] else slopes[-1]
        return p[1]+(x-p[0])*m
    def along(x,offset,z):
        m=(axis_y(x+.05)-axis_y(x-.05))/.1
        length=math.hypot(m,1)
        return (x-m/length*offset,axis_y(x)+offset/length,z)
    def transition(x,a,b):
        t=max(0.,min(1.,(a-x)/(a-b)))
        return t*t*(3-2*t)
    def parallel_path(a,b,offset,z):
        count=max(2,math.ceil(abs(b-a)/30))
        return [along(x,offset(x) if callable(offset) else offset,z(x) if callable(z) else z)
                for x in (a+(b-a)*i/count for i in range(count+1))]
    def ground_offset(x):
        return 17.875+1.375*transition(x,4230,4020)+9.75*transition(x,3915,3659)
    def gwacheon_offset(x):
        # 고가 진입 차로 아래를 한 번 지나 지상 3차로 옆으로 이어진다.
        return 8.125+4.25*transition(x,4230,4020)+9.375*transition(x,3915,3659)
    # transition()의 기본 진행 방향과 반대인 동쪽 방향에는 선형 보간을 쓴다.
    def east_blend(x,a,b):
        t=max(0.,min(1.,(x-a)/(b-a)))
        return t*t*(3-2*t)
    def surface_return_offset(x): return layout["return_surface_offset"]-4.25*east_blend(x,4338,4610)
    def elevated_return_offset(x): return layout["return_offset"]-5.125*east_blend(x,4335,4610)
    aligned_nodes={
        "sn_merge":along(3915,5.25,8),"sn_tg_fan":along(3659,9.75,8),
        "sn_tg_gate_in":along(3550,9.75,8),"sn_tg_gate_out":along(3505,9.75,8),
        "sn_tg_rejoin":along(3387,9.75,8),"sn_local_w":along(3458,ground_offset(3458),0),
        "sn_return_split":along(3916,layout["return_offset"],8),
        "sn_return_merge":along(4335,layout["return_offset"],8),
        "sn_local_bridge":along(3615,layout["return_surface_offset"],0),
        "sn_local_e":along(4000,layout["return_surface_offset"],0),
        "sn_local_merge":along(4338,surface_return_offset(4338),0),
        "sn_local_east_end":along(4550,surface_return_offset(4550),0),
        "sn_gwacheon_end":along(boundary[0],layout["gwacheon_offset"],0),
    }
    for n in nodes:
        if n.get("id") in aligned_nodes:
            for key,value in zip(("x","y","z"),aligned_nodes[n.get("id")]): n.set(key,str(value))
    parallel_shapes={
        "seonam_lane_prepare":parallel_path(4610,4313,0,0),
        "seonam_surface_w":parallel_path(4313,3458,ground_offset,0),
        "seonam_approach":parallel_path(4313,3915,lambda x:11.375-2.625*transition(x,4313,3915),lambda x:8*transition(x,4313,3915)),
        "seonam_tg_direct":parallel_path(4313,3915,lambda x:3.25+.25*transition(x,4313,3915),lambda x:8*transition(x,4313,3915)),
        "seonam_gwacheon":parallel_path(4313,boundary[0],gwacheon_offset,
                                       lambda x:-6*transition(x,4313,4230)+6*transition(x,3880,3700)),
        "seonam_main":parallel_path(3915,3659,5.25,8),
        "seonam_tg_fan":parallel_path(3659,3550,9.75,8),
        "seonam_tg_gate":parallel_path(3550,3505,9.75,8),
        "seonam_tg_rejoin":parallel_path(3505,3387,9.75,8),
        "seonam_after_tg":parallel_path(3387,3105,lambda x:5.25*(1-transition(x,3387,3105)),8),
        "seonam_return": [node_point("gangnam_east_return"),(3190,-1714,8)]+parallel_path(3300,3916,layout["return_offset"],8),
        "seonam_return_main":parallel_path(3916,4335,layout["return_offset"],8),
        "seonam_return_east":parallel_path(4335,4610,elevated_return_offset,lambda x:8*(1-east_blend(x,4335,4610))),
        "seonam_surface_e":parallel_path(3615,4000,layout["return_surface_offset"],0),
        "seonam_surface_e2":parallel_path(4000,4338,surface_return_offset,0),
        "seonam_surface_out":parallel_path(4338,4550,surface_return_offset,0),
        "seonam_local_to_yangjae":parallel_path(4550,4610,surface_return_offset,0),
    }
    for e in edges:
        if e.get("id") in parallel_shapes:
            e.set("shape",shape_string(parallel_shapes[e.get("id")]))
    used_nodes={e.get(key) for e in edges for key in ("from","to")}
    for n in list(nodes):
        if n.get("id") not in used_nodes: nodes.remove(n)
    save(nodes,"sadang.nod.xml")
    save(edges,"sadang.edg.xml")
    save(connections, "sadang.con.xml")

    # 양재대로와 경부 본선의 XY 교차점은 교차로가 아니다.
    # 두 본선은 노드를 공유하지 않으며 고리형/외곽 램프를 통해서만 이동한다.
    yangjae_main = {e.get("id") for e in edges
                    if e.get("id").startswith(("yangjae_daero_", "yj_reverse_"))}
    gyeongbu_main = {e.get("id") for e in edges
                    if e.get("id").startswith("gyeongbu_")
                    and e.get("id") != "gyeongbu_to_yangjae"}
    def road_nodes(edge_ids):
        return {e.get(key) for e in edges if e.get("id") in edge_ids
                for key in ("from", "to")}
    if road_nodes(yangjae_main) & road_nodes(gyeongbu_main):
        raise ValueError("양재IC: 양재대로와 경부 본선은 별도의 높이와 노드로 분리해야 합니다.")
    for connection in connections:
        a, b = connection.get("from"), connection.get("to")
        if ((a in yangjae_main and b in gyeongbu_main)
                or (a in gyeongbu_main and b in yangjae_main)):
            raise ValueError("양재IC: 두 본선 사이 이동은 램프를 거쳐야 합니다.")

    net_file = os.path.join(out_dir, "sadang.net.xml")
    convert_cmd = [checkBinary("netconvert"), "--node-files", os.path.join(out_dir,"sadang.nod.xml"),
                    "--edge-files", os.path.join(out_dir,"sadang.edg.xml"),
                    "--connection-files", os.path.join(out_dir,"sadang.con.xml"),
                    "--no-turnarounds", "true", "-o", net_file]
    def convert():
        result=subprocess.run(convert_cmd,capture_output=True,text=True,encoding="utf8")
        if result.returncode:
            raise RuntimeError(result.stderr)
    convert()
    # 차로 수가 바뀌는 지점의 실제 접속 면을 만든다. endOffset만 쓰면
    # 노면 없이 내부 차로 선만 길게 남으므로, 양쪽 차로 끝의 외곽을
    # SUMO junction 형상으로 연결하고 추가 endOffset은 제거한다.
    built=ET.parse(net_file).getroot()
    dx,dy=map(float,built.find("location").get("netOffset").split(","))
    built_edges={e.get("id"):e for e in built.findall("edge") if e.get("function")!="internal"}
    def hull(ps):
        ps=sorted(set(ps))
        def cross(o,a,b): return (a[0]-o[0])*(b[1]-o[1])-(a[1]-o[1])*(b[0]-o[0])
        lower,upper=[],[]
        for p in ps:
            while len(lower)>1 and cross(lower[-2],lower[-1],p)<=0: lower.pop()
            lower.append(p)
        for p in reversed(ps):
            while len(upper)>1 and cross(upper[-2],upper[-1],p)<=0: upper.pop()
            upper.append(p)
        return lower[:-1]+upper[:-1]
    for eid in road["edge_end_offsets"]:
        # IC 출구의 접속면은 위에서 정확한 차로 횡단면으로 정했다.
        if eid=="yangjae_daero_d": continue
        target=next(e.get("to") for e in edges if e.get("id")==eid)
        boundary=[]
        for e in edges:
            incoming=e.get("to")==target
            if not incoming and e.get("from")!=target: continue
            for lane in built_edges[e.get("id")].findall("lane"):
                ps=[tuple(map(float,p.split(",")[:2])) for p in lane.get("shape").split()]
                a,b=ps[-2:] if incoming else ps[:2]
                px,py=b if incoming else a
                vx,vy=b[0]-a[0],b[1]-a[1]
                size=math.hypot(vx,vy)
                half=float(lane.get("width","3.2"))/2
                for sign in (-1,1):
                    boundary.append((round(px-dx-sign*vy/size*half,3),round(py-dy+sign*vx/size*half,3)))
        node=next(v for v in nodes if v.get("id")==target)
        node.set("shape",shape_string(hull(boundary)))
    for edge in edges:
        edge.attrib.pop("endOffset",None)
    save(nodes,"sadang.nod.xml")
    save(edges,"sadang.edg.xml")
    convert()

    routes = ET.Element("routes")
    ET.SubElement(routes, "vType", id="normal", vClass="passenger", color="1,0.85,0.2")
    ET.SubElement(routes, "vType", id="city_bus", vClass="bus", length="12",
                  color="0.2,0.6,1", guiShape="bus")
    for a in points:
        for movement, dest in [("straight",opposite[a]), ("right",right[a]), ("left",left[a])]:
            rid = f"{a}_{movement}"
            prefix = {"north":"isu_in ", "south":"ic_upstream_in ic_road_in "}.get(a, "")
            suffix = {"north":" isu_out", "south":" ic_road_out ic_upstream_out"}.get(dest, "")
            ET.SubElement(routes, "route", id=rid, edges=f"{prefix}{a}_in {dest}_out{suffix}")
            ET.SubElement(routes, "flow", id=f"flow_{rid}", type="normal", route=rid,
                          begin="0", end="100000", period=str(float(road["flow_period"])*3),
                          departLane="best", departSpeed="0")
    ET.SubElement(routes, "route", id="ic_to_isu", edges="heolleung_start yangjae_ramp yangjae_daero_a yangjae_daero_b yangjae_daero_c yangjae_daero_d seonam_approach seonam_main gangnam_in ic_ramp_in ic_road_in south_in north_out isu_out")
    ET.SubElement(routes, "flow", id="ic_traffic", type="normal", route="ic_to_isu",
                  begin="0", end="100000", period=str(road["flow_period"]),
                  departLane="best", departSpeed="0")
    ET.SubElement(routes, "flow", id="city_bus_north", type="city_bus", route="south_straight",
                  begin="0", end="100000", period="45", departLane="3", departSpeed="0")
    ET.SubElement(routes, "flow", id="city_bus_south", type="city_bus", route="north_straight",
                  begin="0", end="100000", period="60", departLane="3", departSpeed="0")
    ET.SubElement(routes, "route", id="gangnam_through", edges="gangnam_in gangnam_continue")
    ET.SubElement(routes, "flow", id="gangnam_through_flow", type="normal", route="gangnam_through",
                  begin="0", end="100000", period=str(road["gangnam_through_period"]),
                  departLane="free", departSpeed="0")
    ET.SubElement(routes, "route", id="gangnam_return", edges="gangnam_return seonam_return seonam_return_main seonam_return_east")
    ET.SubElement(routes, "flow", id="gangnam_return_flow", type="normal", route="gangnam_return",
                  begin="0", end="100000", period=str(road["gangnam_through_period"]),
                  departLane="free", departSpeed="0")
    for route_id, edge_sequence, period in [
        ("gyeongbu_through", "gyeongbu_in gyeongbu_in_after gyeongbu_midway gyeongbu_out", 8),
        ("gyeongbu_southbound", "gyeongbu_southbound gyeongbu_southbound_b gyeongbu_southbound_c", 8),
        ("gyeongbu_exit", "gyeongbu_in gyeongbu_in_after gyeongbu_to_yangjae yangjae_daero_c yangjae_daero_d seonam_approach seonam_main gangnam_in gangnam_continue", 15),
        ("loop_nw_traffic", "heolleung_start yangjae_ramp yangjae_daero_a yangjae_daero_b yangjae_daero_c loop_nw gyeongbu_southbound_b gyeongbu_southbound_c", 20),
        ("loop_sw_traffic", "gyeongbu_southbound gyeongbu_southbound_b loop_sw yj_reverse_b yj_reverse_c", 20),
        ("loop_se_traffic", "yj_reverse_a yj_reverse_b loop_se gyeongbu_in_after gyeongbu_midway gyeongbu_out", 20),
        ("yangjae_gyeongbu", "heolleung_start yangjae_ramp yangjae_daero_a yangjae_to_gyeongbu gyeongbu_out", 18),
        ("seonam_main_traffic", "seonam_main_in seonam_main gangnam_in gangnam_continue", 10),
        ("seonam_surface_w_traffic", "yangjae_daero_d seonam_surface_w seonam_surface_ramp seonam_underpass_w", 15),
        ("seonam_surface_e_traffic", "seonam_underpass_e seonam_surface_e seonam_surface_e2 seonam_surface_out", 15),
        ("seonam_entry_traffic", "seonam_underpass_e seonam_surface_e seonam_local_entry seonam_return_east", 20),
        ("seonam_exit_traffic", "gangnam_return seonam_return seonam_return_exit seonam_surface_out", 20),
        ("umyeon_south_traffic", "umyeon_south", 12),
        ("umyeon_north_traffic", "umyeon_north", 12),
        ("umyeon_exit_traffic", "umyeon_exit", 25),
        ("umyeon_entry_traffic", "yangjae_daero_d seonam_surface_w seonam_surface_ramp umyeon_entry", 25),
        ("umyeon_yangjae_traffic", "umyeon_yangjae_exit seonam_surface_e seonam_surface_e2 seonam_surface_out", 25),
        ("isu_cross_east", "isu_west_in isu_east_out", 18),
        ("isu_cross_west", "isu_east_in isu_west_out", 18),
        ("isu_turn_north", "isu_west_in isu_finish_out", 35),
    ]:
        ET.SubElement(routes, "route", id=route_id, edges=edge_sequence)
        ET.SubElement(routes, "flow", id=f"{route_id}_flow", type="normal", route=route_id,
                      begin="0", end="100000", period=str(period), departLane="best", departSpeed="0")
    # 차로 변화 지점을 나눠도 모든 배경 차량이 같은 연속 경로를 이용한다.
    def expand_edge(eid,seen=frozenset()):
        if eid in seen:
            raise ValueError(f"경로 확장 설정이 순환합니다: {eid}")
        result=[]
        for part in road["seonam"]["route_expansions"].get(eid,[eid]):
            result.extend([part] if part==eid else expand_edge(part,seen|{eid}))
        return result
    for route in routes.findall("route"):
        expanded = []
        original=route.get("edges").split()
        if original[0]=="isu_in":
            expanded.append("isu_finish_in")
        for edge_id in original:
            expanded.extend(expand_edge(edge_id))
        if original[-1]=="isu_out":
            expanded.append("isu_finish_out")
        if route.get("id")=="loop_se_traffic":
            expanded = ["seonam_underpass_e","seonam_surface_e","seonam_surface_e2",
                        "seonam_surface_out","seonam_local_to_yangjae"] + expanded
        # 지상도로/우면산로 방향 차량도 양재IC 중간에서 생성하지 않는다.
        # 기존 분기 경로는 유지하고 공공도로 진입부부터 주행해 들어온다.
        if expanded[0]=="yangjae_daero_d":
            expanded = ["heolleung_start","yangjae_ramp","yangjae_daero_a",
                        "yangjae_daero_b","yangjae_daero_c"] + expanded
        inserts={
            ("yangjae_daero_d","seonam_lane_prepare"):["mh_wb_e","mh_wb_w"],
            ("seonam_main_in","seonam_main"):["mh_wb_e","mh_wb_w","seonam_lane_prepare","seonam_approach"],
            ("seonam_return_east","yangjae_central_return"):["mh_eb_w","mh_eb_e"],
            ("seonam_local_to_yangjae","yj_reverse_a"):["mh_eb_w","mh_eb_e"],
        }
        continuous=[]
        for index,eid in enumerate(expanded):
            if index: continuous.extend(inserts.get((expanded[index-1],eid),[]))
            continuous.append(eid)
        expanded=continuous
        route.set("edges", " ".join(expanded))
    # 짧은 매헌로 가지: 반대쪽 가지를 만들지 않는 3갈래 교차로.
    route_map={r.get("id"):r.get("edges").split() for r in routes.findall("route")}
    wb_prefix=route_map["ic_to_isu"]
    eb_prefix=route_map["gangnam_return"]
    for rid,ps in [
            ("mh_west_to_side",wb_prefix[:wb_prefix.index("mh_wb_e")+1]+["mh_side_out"]),
            ("mh_east_to_side",eb_prefix[:eb_prefix.index("mh_eb_w")+1]+["mh_side_out"]),
            ("mh_side_to_west",["mh_side_in","mh_wb_w","seonam_lane_prepare","seonam_surface_w","seonam_surface_ramp","seonam_underpass_w"]),
            ("mh_side_to_east",["mh_side_in","mh_eb_e","yj_reverse_a","yj_reverse_b","yj_reverse_c","yangjae_ramp_return","heolleung_return"])]:
        ET.SubElement(routes,"route",id=rid,edges=" ".join(ps))
        ET.SubElement(routes,"flow",id=rid+"_flow",type="normal",route=rid,
                      begin="0",end="100000",period="45",departLane="best",departSpeed="0")
    # 새 사거리에서 나눈 본선 구간을 모든 기존 경로에 함께 반영한다.
    for route in routes.findall("route"):
        ps=[]
        for eid in route.get("edges").split():
            ps.extend({"mh_wb_w":["mh_wb_w","mh_wb_after"],
                       "mh_eb_w":["mh_eb_before","mh_eb_w"]}.get(eid,[eid]))
        if route.get("id")=="seonam_main_traffic":
            ps=["seonam_tg_direct" if eid=="seonam_approach" else eid for eid in ps]
        if route.get("id")=="umyeon_entry_traffic":
            ps=ps[:ps.index("seonam_lane_prepare")+1]+["seonam_gwacheon","umyeon_entry","umyeon_south_after"]
        elif "seonam_surface_w" in ps:
            ps=ps[:ps.index("seonam_surface_w")+1]+["seonam_banpo_ground"]
        route.set("edges"," ".join(ps))
    road_routes={r.get("id"):r.get("edges").split() for r in routes.findall("route")}
    west=road_routes["ic_to_isu"];east=road_routes["gangnam_return"]
    cross_routes=[]
    for direction in ("n","s"):
        cross_routes.extend([
            ("cross_west_to_"+direction,west[:west.index("mh_wb_w")+1]+["mh_cross_"+direction+"_out"]),
            ("cross_east_to_"+direction,east[:east.index("mh_eb_before")+1]+["mh_cross_"+direction+"_out"]),
            ("cross_"+direction+"_to_west",["mh_cross_"+direction+"_in"]+west[west.index("mh_wb_after"):]),
            ("cross_"+direction+"_to_east",["mh_cross_"+direction+"_in"]+east[east.index("mh_eb_w"):]),
            ("cross_"+direction+"_straight",["mh_cross_"+direction+"_in","mh_cross_"+("s" if direction=="n" else "n")+"_out"]),
        ])
    for rid,ps in cross_routes:
        ET.SubElement(routes,"route",id=rid,edges=" ".join(ps))
        ET.SubElement(routes,"flow",id=rid+"_flow",type="normal",route=rid,
                      begin="0",end="100000",period="60",departLane="best",departSpeed="0")
    # 없앤 연결로를 사용하던 배경차는 기존의 연속 본선/지상도로로 보낸다.
    route_paths={r.get("id"):r.get("edges").split() for r in routes.findall("route")}
    for route in routes.findall("route"):
        replacement={"seonam_entry_traffic":"seonam_surface_e_traffic",
                     "seonam_exit_traffic":"gangnam_return"}.get(route.get("id"))
        if replacement: route.set("edges"," ".join(route_paths[replacement]))
    deleted_routes=set()
    for route in list(routes.findall("route")):
        ps=route.get("edges").split()
        if "seonam_gwacheon" in ps:
            ps=ps[:ps.index("seonam_gwacheon")+1]
        elif "seonam_surface_w" in ps:
            ps=ps[:ps.index("seonam_surface_w")+1]
        elif route.get("id").startswith("umyeon_"):
            deleted_routes.add(route.get("id"));routes.remove(route);continue
        else:
            while ps and ps[0] in removed_edges: ps.pop(0)
        if not ps or any(e in removed_edges for e in ps):
            deleted_routes.add(route.get("id"));routes.remove(route)
        else: route.set("edges"," ".join(ps))
    for flow in list(routes.findall("flow")):
        if flow.get("route") in deleted_routes: routes.remove(flow)
    save(routes, "sadang.rou.xml")
    # 중앙 정류장 위치는 공개 정류장 좌표와 로드뷰를 기준으로 배치.
    from sumolib.net import readNet
    preview_net = readNet(net_file)
    additional=ET.Element("additional")
    # trueZ에서 SUMO의 기본 교차로 면은 고정 표시 높이에 남아 차량을 덮는다.
    # 같은 교차로 외곽을 실제 노면 높이에 그려 차량이 그 위로 지나가게 한다.
    pavement_net=ET.parse(net_file).getroot()
    low=tuple(map(int,road["level_colors"]["lower"].split(",")))
    high=tuple(map(int,road["level_colors"]["upper"].split(",")))
    for junction in pavement_net.findall("junction"):
        outline=junction.get("shape","").split()
        if junction.get("type")=="internal" or len(outline)<3: continue
        z=float(junction.get("z","0"))
        ratio=max(0.,min(1.,z/geometry["gyeongbu_height"]))
        color=",".join(str(round(a+(b-a)*ratio)) for a,b in zip(low,high))
        # polygon layer가 높이를 결정하므로 외곽 좌표는 XY만 사용한다.
        shape=" ".join(",".join(p.split(",")[:2]) for p in outline)
        if junction.get("id")=="mh_east" and carriageway_shift:
            # 접속면 가운데도 녹지 간격을 유지한다. 두 진행 방향의 노면만 그린다.
            center=(mh["east"][0]+dx,mh["east"][1]+dy)
            for sign in (-1,1):
                polygon=[tuple(map(float,p.split(",")[:2])) for p in outline]
                for limit,greater in ((carriageway_shift,True),(carriageway_shift+4*lane_width,False)):
                    clipped=[]
                    def side(p): return sign*((p[0]-center[0])*rx+(p[1]-center[1])*ry)
                    for a,b in zip(polygon,polygon[1:]+polygon[:1]):
                        da,db=side(a),side(b)
                        ia=da>=limit if greater else da<=limit
                        ib=db>=limit if greater else db<=limit
                        if ia: clipped.append(a)
                        if ia!=ib:
                            t=(limit-da)/(db-da)
                            clipped.append((a[0]+t*(b[0]-a[0]),a[1]+t*(b[1]-a[1])))
                    polygon=clipped
                if len(polygon)>=3:
                    ET.SubElement(additional,"poly",id="junction_pavement_mh_east_"+str(sign),
                                  color=color,fill="true",layer=str(z+.02),shape=shape_string(polygon))
            continue
        ET.SubElement(additional,"poly",id="junction_pavement_"+junction.get("id"),
                      color=color,fill="true",layer=str(z+.02),shape=shape)
    # trueZ에서도 기본 흰색 차로 바탕은 고정 레이어에 그려져 노면을 덮는다.
    # 실제 노면 높이 바로 위에 얇은 경계선과 차로 점선을 별도로 그린다.
    def road_line(identifier, ps, width, color, layer):
        if len(ps)<2: return
        ET.SubElement(additional,"poly",id=identifier,color=color,fill="false",
                      lineWidth=str(width),layer=str(layer),
                      shape=shape_string([p[:2] for p in ps]))
    from sumolib.geomhelper import positionAtShapeOffset
    # 기존 차로 경계에만 작은 통행구 분리대를 그린다.
    toll = road["seonam_toll"]
    gate = preview_net.getEdge("seonam_tg_gate")
    for lane in gate.getLanes():
        line = lane.getShape3D()
        midpoint = lane.getLength() * .55
        half = toll["island_length"] / 2
        segment = [positionAtShapeOffset(line, midpoint-half),
                   positionAtShapeOffset(line, midpoint+half)]
        sides = (1, -1) if lane.getIndex() == len(gate.getLanes())-1 else (1,)
        for side in sides:
            boundary = move2side([p[:2] for p in segment], side * lane.getWidth()/2)
            road_line("toll_island_"+lane.getID()+str(side), boundary, .45,
                      "205,205,195", min(p[2] for p in line)+.12)
    for e in preview_net.getEdges():
        for lane in e.getLanes():
            # 입체 합류·진출 연결부에는 별도 긴 테두리를 덧그리지 않는다.
            # 인접 본선 차선 위로 흰색 선이 겹치는 것을 막는다.
            if e.getID() in {"seonam_local_entry", "seonam_return_exit"}:
                continue
            ps=lane.getShape3D()
            level=min(p[2] for p in ps)+0.05
            half=lane.getWidth()/2
            for side in (-1,1):
                boundary=move2side([p[:2] for p in ps],side*half)
                if (side==1 and lane.getIndex()==0) or (side==-1 and lane.getIndex()==len(e.getLanes())-1):
                    road_line("border_"+lane.getID()+str(side),boundary,.12,"235,235,235",level)
                elif side==-1:
                    total=sum(math.dist(a,b) for a,b in zip(boundary,boundary[1:]))
                    for i in range(0,int(total),9):
                        # 실제 차로 경계를 따라 점선을 배치한다.
                        sub=[positionAtShapeOffset(boundary,float(i)),
                             positionAtShapeOffset(boundary,min(total,i+3.))]
                        road_line("dash_"+lane.getID()+"_"+str(i),sub,.12,"235,235,235",level)
        # 중앙 통과도로 바깥에 분리대를 표시한다. 도로 연결은 추가하지 않는다.
        if e.getID() in ("seonam_main_in","yangjae_central_return"):
            for side in (-1,1):
                outer=e.getLanes()[0] if side==1 else e.getLanes()[-1]
                ps=outer.getShape3D()
                boundary=move2side([p[:2] for p in ps],side*(outer.getWidth()/2+.20))
                road_line("divider_"+e.getID()+str(side),boundary,.35,"170,170,160",min(p[2] for p in ps)+.1)
    # 지하차도: 실제로 낮아지는 차로 위에 지표면 높이의 옹벽과 덮개를 둔다.
    # 지상 측도와 연결하지 않으며, 낮은 차량만 덮개 아래에서 가려진다.
    underpass=road["yangjae_underpass"]
    for eid in ("seonam_main_in","yangjae_central_return"):
        e=preview_net.getEdge(eid)
        # 직선 진입부의 지하 구간: 기존 교량 덮개와 별도로 추가한다.
        # 바깥 지상 차로는 그대로 두고 중앙 통과도로만 지표 아래로 덮는다.
        x0,x1=underpass["entry_cover_x"]
        centers=[]
        axis=e.getShape()
        for target_x in (x0+dx,x1+dx):
            for a,b in zip(axis,axis[1:]):
                if min(a[0],b[0])<=target_x<=max(a[0],b[0]) and abs(b[0]-a[0])>1e-6:
                    t=(target_x-a[0])/(b[0]-a[0])
                    centers.append((target_x,a[1]+t*(b[1]-a[1])))
                    break
        if len(centers)!=2:
            raise ValueError("양재 직선부 지하차도 입구가 중앙 도로와 만나지 않습니다.")
        half_width=sum(l.getWidth() for l in e.getLanes())/2+.35
        side_a=move2side(centers,half_width)
        side_b=move2side(centers,-half_width)
        ET.SubElement(additional,"poly",id="entry_tunnel_roof_"+eid,
                      color="115,115,110",fill="true",layer="0.5",
                      shape=shape_string(side_a+list(reversed(side_b))))
        for i in (0,1):
            road_line("entry_tunnel_mouth_"+eid+str(i),[side_a[i],side_b[i]],
                      1.2,"205,205,195",.55)
        for i,side in enumerate((side_a,side_b)):
            road_line("entry_tunnel_parapet_"+eid+str(i),side,.5,"185,185,175",.55)
        for side in (-1,1):
            outer=e.getLanes()[0] if side==1 else e.getLanes()[-1]
            trench=[p for p in outer.getShape3D() if p[2]<-.4 and p[0]-dx>5000]
            if len(trench)>1:
                boundary=move2side([p[:2] for p in trench],side*(outer.getWidth()/2+.15))
                road_line("underpass_wall_"+eid+str(side),boundary,
                          underpass["wall_width"],"185,185,175",.25)
                shadow=move2side(boundary,-side*.45)
                road_line("underpass_wall_shadow_"+eid+str(side),shadow,.3,"20,20,20",.26)
        # 경부고속도로 교량과 만나는 지점만 덮는다. 나머지 진입부는 열린 절개부다.
        axis=e.getShape()
        bridge=preview_net.getEdge("gyeongbu_in_after").getShape()
        crossing=None
        for a,b in zip(axis,axis[1:]):
            for c,d in zip(bridge,bridge[1:]):
                ux,uy=b[0]-a[0],b[1]-a[1]
                vx,vy=d[0]-c[0],d[1]-c[1]
                den=ux*vy-uy*vx
                if abs(den)<1e-6: continue
                t=((c[0]-a[0])*vy-(c[1]-a[1])*vx)/den
                s=((c[0]-a[0])*uy-(c[1]-a[1])*ux)/den
                if 0<=t<=1 and 0<=s<=1:
                    norm=math.hypot(ux,uy)
                    crossing=(a[0]+t*ux,a[1]+t*uy,ux/norm,uy/norm)
        if crossing:
            x,y,ux,uy=crossing
            half_width=sum(l.getWidth() for l in e.getLanes())/2+.5
            half_length=underpass["portal_half_length"]
            roof=[(x+along*ux+across*uy,y+along*uy-across*ux)
                  for along,across in [(-half_length,-half_width),(-half_length,half_width),
                                        (half_length,half_width),(half_length,-half_width)]]
            ET.SubElement(additional,"poly",id="underpass_cover_"+eid,
                          color="125,125,120",fill="true",layer="0.35",shape=shape_string(roof))
            for sign in (-1,1):
                beam=[(x+sign*half_length*ux+across*uy,
                       y+sign*half_length*uy-across*ux) for across in (-half_width,half_width)]
                road_line("underpass_portal_"+eid+str(sign),beam,.8,"195,195,185",.4)
    for stop_id,lane,start,end,label in [
            ("isu_median_north","isu_out_3",140,175,"Isu median northbound"),
            ("post_office_south","isu_in_3",285,320,"Sadang post office southbound")]:
        ET.SubElement(additional,"busStop",id=stop_id,lane=lane,startPos=str(start),
                      endPos=str(end),friendlyPos="true",name=label)
    for label,node in [("Yangjae Start","yangjae_start"),("Yangjae IC","yangjae_merge"),
                       ("Seonam TG","sn_tg_gate_in"),("Sadang IC","south_end"),
                       ("Sadang Station","center"),("Isu Station","isu")]:
        x,y=preview_net.getNode(node).getCoord()
        ET.SubElement(additional,"poi",id=label.replace(" ","_"),x=str(x),y=str(y+55),
                      color="255,180,50",layer="150",width="5",height="5")
    save(additional,"sadang.stops.add.xml")
    for flow in routes.findall("flow"):
        if flow.get("id")=="city_bus_north":
            ET.SubElement(flow,"stop",busStop="isu_median_north",duration="12")
        elif flow.get("id")=="city_bus_south":
            ET.SubElement(flow,"stop",busStop="post_office_south",duration="12")
    # 통행구를 지나는 차량을 차로별 경로로 나눈다. 전체 유입량은 유지한다.
    # 통행구 차로 배정과 정차 시간은 실제 운영 기록이 아닌 모형 가정이다.
    import copy
    toll_routes = {r.get("id"): r for r in routes.findall("route")
                   if "seonam_tg_gate" in r.get("edges", "").split()}
    lane_count = len(gate.getLanes())
    variants = {}
    for rid, original in toll_routes.items():
        for index in range(lane_count):
            variant = copy.deepcopy(original)
            variant_id = f"{rid}_toll_{index}"
            variant.set("id", variant_id)
            lane = gate.getLane(index)
            attrs = {"lane": lane.getID(), "endPos": str(round(lane.getLength()*.55, 2))}
            if index in toll["cash_lanes"]:
                attrs["duration"] = str(toll["wait_seconds"])
            else:
                attrs["speed"] = str(toll["hipass_speed"])
            ET.SubElement(variant, "stop", **attrs)
            routes.append(variant)
            variants[(rid, index)] = variant_id
    for flow in list(routes.findall("flow")):
        rid = flow.get("route")
        if rid not in toll_routes:
            continue
        period, begin = float(flow.get("period")), float(flow.get("begin", "0"))
        routes.remove(flow)
        for index in range(lane_count):
            vehicle_flow = copy.deepcopy(flow)
            vehicle_flow.set("id", f"{flow.get('id')}_toll_{index}")
            vehicle_flow.set("route", variants[(rid, index)])
            vehicle_flow.set("begin", str(begin + index * period))
            vehicle_flow.set("period", str(period * lane_count))
            routes.append(vehicle_flow)
    # SUMO는 차량 흐름을 출발 시간 순서로 읽는다.
    ordered_flows = sorted(routes.findall("flow"), key=lambda f: float(f.get("begin", "0")))
    for flow in ordered_flows:
        routes.remove(flow)
    routes.extend(ordered_flows)
    save(routes, "sadang.rou.xml")
    view = ET.Element("viewsettings")
    # SUMO의 기본 2D 표시는 높이가 다른 도로도 같은 검정색으로 그린다.
    # 실제 lane의 z좌표로 색을 정하므로 연결로의 오르내림도 함께 표시된다.
    scheme = ET.SubElement(view, "scheme", name="real world")
    # 도로와 차량을 종류별 우선순위 대신 실제 z 순서로 표시한다.
    ET.SubElement(scheme, "opengl", trueZ="true")
    ET.SubElement(scheme,"background",backgroundColor=road["background_color"])
    ET.SubElement(scheme,"pois",poiName_show="true",poiName_size="35",poiName_color="255,230,160")
    colors = road["level_colors"]
    edge_view = ET.SubElement(scheme, "edges", laneEdgeMode="22", laneShowBorders="false")
    junction_view = ET.SubElement(scheme, "junctions", junctionMode="3", drawShape="false")
    for parent, name in [(edge_view, "by height at geometry-segment start"),
                         (junction_view, "by height")]:
        palette = ET.SubElement(parent, "colorScheme", name=name, interpolated="true")
        ET.SubElement(palette, "entry", color=colors["lower"], threshold="-12")
        ET.SubElement(palette, "entry", color=colors["lower"], threshold="0")
        ET.SubElement(palette, "entry", color=colors["upper"],
                      threshold=str(geometry["gyeongbu_height"]))
    # 사당IC 진출로의 분기·합류 구간을 처음부터 확대해 보여준다.
    xmin, ymin, xmax, ymax = preview_net.getBoundary()
    if road.get("preview_focus") == "gyeongbu_extension":
        ax, ay = preview_net.getNode("gyeongbu_mid").getCoord()
        bx, by = preview_net.getNode("gyeongbu_end").getCoord()
        center_x, center_y = (ax+bx)/2, (ay+by)/2
        zoom = 100 * max(xmax-xmin, ymax-ymin) / 1400
    elif road.get("preview_focus") == "seonam_tg":
        ax, ay = preview_net.getNode("sn_tg_gate_in").getCoord()
        bx, by = preview_net.getNode("sn_tg_gate_out").getCoord()
        center_x, center_y = (ax+bx)/2, (ay+by)/2
        zoom = 100 * max(xmax-xmin, ymax-ymin) / 450
    elif road.get("preview_focus") == "maeheon_junction":
        center_x,center_y=mh["junction"][0]+dx,mh["junction"][1]+dy
        zoom=100*max(xmax-xmin,ymax-ymin)/900
    elif road.get("preview_focus") == "yangjae_underpass":
        center_x,center_y=5314+dx,-1442+dy
        zoom=100*max(xmax-xmin,ymax-ymin)/650
    elif road.get("preview_focus") == "seonam_split":
        ax, ay = preview_net.getNode("yangjae_daero_end").getCoord()
        bx, by = preview_net.getNode("sn_merge").getCoord()
        center_x, center_y = (ax+bx)/2, (ay+by)/2
        zoom = 100 * max(xmax-xmin, ymax-ymin) / 1500
    elif road.get("preview_focus") == "seonam_ic":
        ax, ay = preview_net.getNode("sn_merge").getCoord()
        bx, by = preview_net.getNode("um_north_out").getCoord()
        center_x, center_y = (ax+bx)/2, (ay+by)/2 - 80
        zoom = 100 * max(xmax-xmin, ymax-ymin) / 1600
    elif road.get("preview_focus") == "yangjae_ic":
        ax, ay = preview_net.getNode("g_north_lower").getCoord()
        bx, by = preview_net.getNode("gyeongbu_mid").getCoord()
        center_x, center_y = (ax+bx)/2, (ay+by)/2
        zoom = 100 * max(xmax-xmin, ymax-ymin) / 900
    elif road.get("preview_focus") == "sadang_ic":
        ax, ay = preview_net.getNode("ic_ramp_start").getCoord()
        bx, by = preview_net.getNode("south_end").getCoord()
        center_x, center_y = (ax+bx)/2, (ay+by)/2
        zoom = 100 * max(xmax-xmin, ymax-ymin) / 900
    else:
        center_x, center_y, zoom = (xmin+xmax)/2, (ymin+ymax)/2, 110
    ET.SubElement(view, "viewport", x=str(center_x), y=str(center_y), zoom=str(zoom))
    ET.SubElement(view, "delay", value="100")
    save(view, "sadang.gui.xml")
    cfg = ET.Element("configuration")
    inp = ET.SubElement(cfg, "input")
    ET.SubElement(inp, "net-file", value="sadang.net.xml")
    ET.SubElement(inp, "route-files", value="sadang.rou.xml")
    ET.SubElement(inp, "additional-files", value="sadang.stops.add.xml")
    ET.SubElement(ET.SubElement(cfg, "gui_only"), "gui-settings-file", value="sadang.gui.xml")
    save(cfg, "sadang.sumocfg")
    if road.get("yangjae_only", False):
        return _build_yangjae_only(out_dir)
    return os.path.join(out_dir, "sadang.sumocfg")


def _build_yangjae_only(out_dir):
    """원본 도로를 보존하며 양재IC 주변만 별도의 SUMO 도로망으로 만든다."""
    import copy
    import xml.etree.ElementTree as ET

    source_net = os.path.join(out_dir, "sadang.net.xml")
    original = ET.parse(source_net).getroot()
    prefixes = ("heolleung_", "yangjae_", "gyeongbu_", "yj_reverse_", "loop_")
    keep = {e.get("id") for e in original.findall("edge")
            if e.get("function") != "internal"
            and (e.get("id", "").startswith(prefixes) or e.get("id") == "seonam_main_in")}
    keep_file = os.path.join(out_dir, "yangjae.keep.txt")
    with open(keep_file, "w", encoding="utf-8") as f:
        f.write("\n".join(sorted(keep)))
    net_file = os.path.join(out_dir, "yangjae.net.xml")
    result = subprocess.run([checkBinary("netconvert"), "--sumo-net-file", "sadang.net.xml",
                             "--keep-edges.input-file", "yangjae.keep.txt", "--keep-edges.postload", "true",
                             "--offset.disable-normalization", "true", "--no-turnarounds", "true",
                             "--no-warnings", "true", "-o", "yangjae.net.xml"],
                            cwd=out_dir, capture_output=True, text=True, encoding="utf-8")
    if result.returncode:
        raise RuntimeError(result.stderr)
    # 잘라내기 전의 차로 형상과 높이를 그대로 복사해 IC 선형을 보존한다.
    reduced_tree = ET.parse(net_file)
    reduced = reduced_tree.getroot()
    original_edges = {e.get("id"): e for e in original.findall("edge")}
    for edge in reduced.findall("edge"):
        previous = original_edges.get(edge.get("id"))
        if previous is None:
            continue
        old_lanes = {l.get("id"): l for l in previous.findall("lane")}
        for lane in edge.findall("lane"):
            old = old_lanes.get(lane.get("id"))
            if old is not None:
                for key in ("shape", "length"):
                    if key in old.attrib:
                        lane.set(key, old.get(key))
    reduced_tree.write(net_file, encoding="utf-8", xml_declaration=True)

    source_routes = ET.parse(os.path.join(out_dir, "sadang.rou.xml")).getroot()
    output_routes = ET.Element("routes")
    for vehicle_type in source_routes.findall("vType"):
        output_routes.append(copy.deepcopy(vehicle_type))
    periods = {"gyeongbu_through": 8, "gyeongbu_southbound": 8,
               "gyeongbu_exit": 15, "loop_nw_traffic": 20, "loop_sw_traffic": 20,
               "loop_se_traffic": 20, "yangjae_gyeongbu": 18, "ic_to_isu": 12,
               "gangnam_return": 12, "seonam_main_traffic": 12}
    for rid, period in periods.items():
        route = source_routes.find(f"route[@id='{rid}']")
        groups, group = [], []
        for eid in route.get("edges").split():
            if eid in keep:
                group.append(eid)
            elif group:
                groups.append(group)
                group = []
        if group:
            groups.append(group)
        if not groups:
            continue
        segment = max(groups, key=len)
        ET.SubElement(output_routes, "route", id=rid, edges=" ".join(segment))
        ET.SubElement(output_routes, "flow", id=rid+"_flow", type="normal", route=rid,
                      begin="0", end="100000", period=str(period),
                      departLane="best", departSpeed="0")
    ET.indent(output_routes)
    ET.ElementTree(output_routes).write(os.path.join(out_dir, "yangjae.rou.xml"),
                                        encoding="utf-8", xml_declaration=True)

    # 남은 도로의 노면 표시·지하차도 외곽만 남기고 다른 지역의 배경 표시를 제외한다.
    old_add = ET.parse(os.path.join(out_dir, "sadang.stops.add.xml")).getroot()
    extra = ET.Element("additional")
    kept_nodes = {j.get("id") for j in reduced.findall("junction") if not j.get("id", "").startswith(":")}
    for item in old_add:
        identifier = item.get("id", "")
        if item.tag == "poly":
            matched_edge = any(("_"+eid+"_") in identifier or identifier.endswith("_"+eid)
                               or ("_"+eid+"-") in identifier for eid in keep)
            junction = identifier.removeprefix("junction_pavement_")
            if matched_edge or (identifier.startswith("junction_pavement_") and junction in kept_nodes):
                extra.append(copy.deepcopy(item))
        elif item.tag == "poi" and identifier in {"Yangjae_Start", "Yangjae_IC"}:
            extra.append(copy.deepcopy(item))
    ET.ElementTree(extra).write(os.path.join(out_dir, "yangjae.stops.add.xml"),
                                encoding="utf-8", xml_declaration=True)
    gui = ET.parse(os.path.join(out_dir, "sadang.gui.xml"))
    x1, y1, x2, y2 = map(float, reduced.find("location").get("convBoundary").split(","))
    gui.find("viewport").set("zoom", str(100*max(x2-x1, y2-y1)/1400))
    gui.write(os.path.join(out_dir, "yangjae.gui.xml"), encoding="utf-8", xml_declaration=True)
    cfg = ET.Element("configuration")
    inp = ET.SubElement(cfg, "input")
    ET.SubElement(inp, "net-file", value="yangjae.net.xml")
    ET.SubElement(inp, "route-files", value="yangjae.rou.xml")
    ET.SubElement(inp, "additional-files", value="yangjae.stops.add.xml")
    ET.SubElement(ET.SubElement(cfg, "gui_only"), "gui-settings-file", value="yangjae.gui.xml")
    cfg_path = os.path.join(out_dir, "yangjae.sumocfg")
    ET.ElementTree(cfg).write(cfg_path, encoding="utf-8", xml_declaration=True)
    return cfg_path


def _build_yangjae_isu(road, ego, out_dir):
    """Build the road-only OSM corridor; no training or behavior scenarios."""
    import json
    import shutil
    import xml.etree.ElementTree as ET
    from sumolib.net import readNet

    source = os.path.join(os.path.dirname(__file__), "maps", "yangjae_isu")
    with open(os.path.join(source, "route.json"), encoding="utf-8") as f:
        route = json.load(f)
    net_path = os.path.join(out_dir, "highway.net.xml")
    shutil.copyfile(os.path.join(source, "corridor.net.xml"), net_path)
    net = readNet(net_path)
    edges = [net.getEdge(e) for e in route["edges"]]
    for first, second in zip(edges, edges[1:]):
        if second not in first.getOutgoing():
            raise ValueError(f"Disconnected route: {first.getID()} -> {second.getID()}")

    def save(root, filename):
        ET.indent(root)
        ET.ElementTree(root).write(os.path.join(out_dir, filename), encoding="utf-8", xml_declaration=True)

    routes = ET.Element("routes")
    ET.SubElement(routes, "vType", id="ego_type", accel=str(ego["accel"]),
                  decel=str(ego["decel"]), maxSpeed=str(ego["max_speed"]), color="1,0,0")
    ET.SubElement(routes, "route", id="r0", edges=" ".join(route["edges"]))
    if road.get("preview_traffic", False):
        ET.SubElement(routes, "vType", id="background", vClass="passenger",
                      guiShape="passenger/sedan", accel="2.6", decel="4.5",
                      maxSpeed="22.22", speedFactor="normc(0.95,0.1,0.7,1.1)",
                      sigma="0.5", color="1,0.85,0.2")
        # Populate all sections immediately with SUMO-controlled passenger cars.
        for index, edge in enumerate(edges):
            if edge.getLength() < 50:
                continue
            for slot in range(max(1, int(edge.getLength() // 180))):
                car = ET.SubElement(routes, "vehicle", id=f"background_{index}_{slot}",
                                    type="background", depart="0", departLane="free",
                                    departPos=str(min(edge.getLength()-10, 10+slot*180)),
                                    departSpeed="max")
                ET.SubElement(car, "route", edges=" ".join(route["edges"][index:]))
        ET.SubElement(routes, "flow", id="background_flow", type="background",
                      route="r0", begin="1", end="86400", period="6",
                      departLane="free", departSpeed="max")
    save(routes, "highway.rou.xml")

    # A thin route overlay makes the intended path visible among side streets.
    additional = ET.Element("additional")
    for i, edge in enumerate(edges):
        if road.get("preview_traffic"):
            continue  # Keep the road surface unobscured when viewing moving cars.
        ET.SubElement(additional, "poly", id=f"route_{i}", color="0.1,0.75,1",
                      fill="false", lineWidth="12", layer="100",
                      shape=" ".join(f"{x:.2f},{y:.2f}" for x, y in edge.getShape()))
    for marker in route["markers"]:
        x, y = marker["xy"]
        ET.SubElement(additional, "poi", id=marker["name"], x=str(x), y=str(y),
                      color="1,0.5,0", layer="101", width="3", height="3")
    save(additional, "highway.scenery.xml")
    xmin, ymin, xmax, ymax = net.getBoundary()
    gui = ET.Element("viewsettings")
    scheme = ET.SubElement(gui, "scheme", name="real world")
    ET.SubElement(scheme, "polys", polyMode="0", polyMinSize="0")
    ET.SubElement(scheme, "pois", poiMode="0", poiMinSize="0")
    cx, cy = route["markers"][3]["xy"] if road.get("preview_traffic") else ((xmin+xmax)/2, (ymin+ymax)/2)
    zoom = 100 * max(xmax-xmin, ymax-ymin) / float(road.get("preview_view_width", 320))
    ET.SubElement(gui, "viewport", zoom=str(zoom) if road.get("preview_traffic") else "95", x=str(cx), y=str(cy))
    ET.SubElement(gui, "delay", value="100")
    save(gui, "highway.gui.xml")
    cfg = ET.Element("configuration")
    inp = ET.SubElement(cfg, "input")
    for key, filename in [("net-file", "highway.net.xml"), ("route-files", "highway.rou.xml"),
                          ("additional-files", "highway.scenery.xml")]:
        ET.SubElement(inp, key, value=filename)
    ET.SubElement(ET.SubElement(cfg, "gui_only"), "gui-settings-file", value="highway.gui.xml")
    save(cfg, "highway.sumocfg")
    return os.path.join(out_dir, "highway.sumocfg")


def _blob(cx: float, cy: float, r: float, rng, n: int = 14,
          roughness: float = 0.35) -> str:
    """유기적인(울퉁불퉁한) 수관 실루엣 다각형을 만든다.

    실제 나무를 위에서 내려다보면 완전한 원이 아니라 잎 뭉치들이
    만드는 불규칙한 테두리다. 꼭짓점마다 반지름을 무작위로 흔들되,
    이웃 꼭짓점과 평균을 내서(smoothing) 뾰족하지 않고 부드럽게
    울퉁불퉁한 실루엣을 얻는다.
    """
    import math
    raw = [1.0 + rng.uniform(-roughness, roughness) for _ in range(n)]
    sm = [(raw[i - 1] + raw[i] + raw[(i + 1) % n]) / 3.0 for i in range(n)]
    pts = []
    for i in range(n):
        a = 2 * math.pi * i / n
        rr = r * sm[i]
        pts.append(f"{cx + rr * math.cos(a):.2f},{cy + rr * math.sin(a):.2f}")
    return " ".join(pts)


def _write_scenery(out_dir: str, road: dict):
    """도로변 나무를 highway.scenery.xml (additional 파일)로 생성.

    항공뷰(top-down) 조경처럼 보이도록 나무 한 그루를 여러 겹으로 그린다:
      layer 1  그림자  : 수관을 남동쪽으로 살짝 밀어낸 반투명 검정 blob
                         (해가 북서쪽에 있는 항공사진 느낌)
      layer 2  수관 몸통: 중간톤 초록의 불규칙 실루엣
      layer 3  잎 뭉치  : 수관 안에 어두운/짙은 클럼프 1~2개
                         → 캐노피의 입체적인 질감
      layer 4  하이라이트: 북서쪽으로 치우친 밝은 초록 작은 blob
    배치도 실제 조경처럼: 단독목 / 2~3그루 군락 / 낮은 관목이 섞인다.
    SCENERY["trees"]가 False면 빈 additional 파일 생성.
    """
    import random
    path = os.path.join(out_dir, "highway.scenery.xml")
    cfg = SCENERY if isinstance(SCENERY, dict) else {}
    if not cfg.get("trees", False):
        with open(path, "w") as f:
            f.write("<additional/>\n")
        return

    rng = random.Random(int(cfg.get("seed", 7)))
    spacing = float(cfg.get("tree_spacing", 45.0))
    jitter = float(cfg.get("tree_jitter", 0.5))
    side = float(cfg.get("side_offset", 6.0))
    length = float(road["length"])
    road_width = 3.2 * int(road["num_lanes"])
    road_bottom = -road_width
    road_top = road_width

    # (몸통, 어두운 클럼프, 밝은 하이라이트) 초록 팔레트 — 수종별 톤
    palettes = [
        ("46,110,44", "32,84,32", "96,158,84"),
        ("38,98,40",  "26,72,28", "84,146,74"),
        ("56,122,50", "40,92,38", "110,168,92"),
        ("30,88,36",  "20,64,26", "76,138,70"),
    ]
    bush_greens = ["70,124,58", "84,136,64", "62,112,52"]

    lines = ["<additional>"]
    tid = [0]

    def draw_tree(x, y, r):
        body, dark, light = rng.choice(palettes)
        i = tid[0]; tid[0] += 1
        # 그림자 (남동쪽 +x, -y 로 오프셋, 반투명)
        lines.append(f'    <poly id="sh_{i}" color="20,26,18,70" fill="1" '
                     f'layer="1" shape="{_blob(x + r*0.35, y - r*0.35, r*1.02, rng)}"/>')
        # 수관 몸통
        lines.append(f'    <poly id="tr_{i}" color="{body}" fill="1" '
                     f'layer="2" shape="{_blob(x, y, r, rng)}"/>')
        # 잎 뭉치 1~2개 (어두운 톤, 수관 내부 무작위 위치)
        for k in range(rng.randint(1, 2)):
            ox = rng.uniform(-0.35, 0.35) * r
            oy = rng.uniform(-0.35, 0.35) * r
            lines.append(f'    <poly id="cl_{i}_{k}" color="{dark}" fill="1" '
                         f'layer="3" shape="{_blob(x + ox, y + oy, r * rng.uniform(0.4, 0.6), rng, n=10)}"/>')
        # 하이라이트 (북서쪽 -x, +y — 그림자 반대 방향)
        lines.append(f'    <poly id="hl_{i}" color="{light}" fill="1" '
                     f'layer="4" shape="{_blob(x - r*0.28, y + r*0.28, r*0.34, rng, n=8, roughness=0.25)}"/>')

    def draw_bush(x, y):
        i = tid[0]; tid[0] += 1
        g = rng.choice(bush_greens)
        r = rng.uniform(1.0, 1.8)
        lines.append(f'    <poly id="bsh_{i}" color="20,26,18,55" fill="1" '
                     f'layer="1" shape="{_blob(x + 0.3, y - 0.3, r, rng, n=8)}"/>')
        lines.append(f'    <poly id="bs_{i}" color="{g}" fill="1" '
                     f'layer="2" shape="{_blob(x, y, r, rng, n=8)}"/>')

    for side_sign, base_y in [
        (+1, road_top + side),
        (-1, road_bottom - side)]:
        x = rng.uniform(0, spacing)
        while x < length:
            y = base_y + side_sign * rng.uniform(0, 4.0)
            roll = rng.random()
            if roll < 0.12:
                draw_bush(x, y)                        # 15%: 낮은 관목
            elif roll < 0.67:
                draw_tree(x, y, rng.uniform(2.6, 4.6))  # 55%: 단독목 (실제 수관 스케일)
            else:
                # 25%: 2~3그루 군락 (겹치듯 붙여 심어 숲 덩어리 느낌)
                n = rng.randint(2, 4)
                for _ in range(n):
                    gx = x + rng.uniform(-5.5, 5.5)
                    gy = y + side_sign * rng.uniform(-2.0, 4.5)
                    draw_tree(gx, gy, rng.uniform(2.2, 3.8))
                x += spacing * 0.4                     # 군락 뒤엔 간격 조금 더
            x += spacing * rng.uniform(1.0 - jitter, 1.0 + jitter)
    lines.append("</additional>")
    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")
