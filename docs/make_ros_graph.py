import re, sys, time
import rclpy
rclpy.init()
n = rclpy.create_node('graph_probe')
time.sleep(3)
HIDE_T = {'/rosout', '/parameter_events', '/tf_static', '/tf', '/bond', '/robot_description', '/clock'}
HIDE_N = re.compile(r'transform_listener|graph_probe|_ros2cli|rviz_navigation_dialog|_rclcpp_node|lifecycle_manager|controller_manager$|^/_')
def ok_t(t):
    return t not in HIDE_T and '/_action/' not in t and not t.endswith(('transition_event', '/costmap_updates', '/costmap_raw_updates', '/costmap_raw', 'published_footprint'))
def nm(i):
    return i.node_namespace.rstrip('/') + '/' + i.node_name
edges_p, edges_s, nodes, topics = set(), set(), set(), set()
for t, _ in n.get_topic_names_and_types():
    if not ok_t(t): continue
    ps = {nm(i) for i in n.get_publishers_info_by_topic(t) if not HIDE_N.search(nm(i))}
    ss = {nm(i) for i in n.get_subscriptions_info_by_topic(t) if not HIDE_N.search(nm(i))}
    if not ps or not ss: continue
    topics.add(t)
    for p in ps: edges_p.add((p, t)); nodes.add(p)
    for s in ss: edges_s.add((t, s)); nodes.add(s)
GAZ = {'/ros_gz_bridge', '/gz_ros_control'}
def q(s): return '"' + s + '"'
o = ['digraph ros {', ' rankdir=LR; bgcolor=white; nodesep=0.3; ranksep=1.1;',
     ' node [fontname="Helvetica", fontsize=12]; edge [color="#5b6475", arrowsize=0.7];']
for x in sorted(nodes):
    col = '#d9f0d9' if x in GAZ else ('#f3e1f5' if x in ('/rviz2','/speed_control','/route_control','/delivery_panel') else '#dbe8f7')
    o.append(f' {q(x)} [shape=box, style="rounded,filled", fillcolor="{col}", color="#0b3d6b"];')
for t in sorted(topics):
    o.append(f' {q("t"+t)} [label={q(t)}, shape=ellipse, style=filled, fillcolor="#fff3d6", color="#c77700", fontsize=10];')
for p, t in sorted(edges_p): o.append(f' {q(p)} -> {q("t"+t)};')
for t, s in sorted(edges_s): o.append(f' {q("t"+t)} -> {q(s)};')
o.append(' subgraph cluster_leg { label="Legend"; fontname="Helvetica"; style=dashed; color="#aaaaaa";'
         ' l1 [label="Simulation (Gazebo bridge / ros2_control)", shape=box, style="rounded,filled", fillcolor="#d9f0d9"];'
         ' l2 [label="Nav2 / localization nodes", shape=box, style="rounded,filled", fillcolor="#dbe8f7"];'
         ' l3 [label="Operator tools (RViz, slider, route, delivery)", shape=box, style="rounded,filled", fillcolor="#f3e1f5"];'
         ' l4 [label="topic", shape=ellipse, style=filled, fillcolor="#fff3d6"]; l1->l2->l3->l4 [style=invis]; }')
o.append('}')
open(sys.argv[1], 'w').write('\n'.join(o))
print(sorted(nodes)); print(len(topics), 'topics')
rclpy.shutdown()
