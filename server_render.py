from flask import Flask, render_template
from IPython.core.display import HTML

import numpy as np
import os
import time

import meshcat
import meshcat.geometry as g
import meshcat.transformations as tf
import io
import contextlib


from flask import Flask, request, redirect, url_for, render_template

from meshcat.animation import Animation
import numpy as np
import pydynorrt as pyrrt
from pydynorrt import pin_more as pyrrt_vis # Small utility functions to visualize motion planning
import pinocchio as pin  # Not needed in the current script (consider removing if not used)
import meshcat
import time
import matplotlib.pyplot as plt
from werkzeug.utils import secure_filename
from flask import Flask, render_template, request, session, send_file
import zipfile
import json
import pathlib
import shutil
import threading


app = Flask(__name__)
app.secret_key = 'your_secret_key'  # Choose a secure key

# Create an IPython HTML object
additional_content = HTML('<p>This is additional content.</p>')
# Set the path for the uploads
UPLOAD_FOLDER = 'uploads'
app.config['UPLOAD_FOLDER'] = UPLOAD_FOLDER

def delete_temp_files(path):
    """Delete files after a delay."""
    time.sleep(10)  # Delay for 10 seconds (or choose an appropriate delay)
    print("deleting ", path)
    if os.path.isfile(path):
        os.remove(path)
    elif os.path.isdir(path):
        shutil.rmtree(path)




@app.route('/download_problem')
def download_problem():
    # Define the directory containing the files you want to include
    problem_files_dir = 'tmp_files_dir'
    pathlib.Path(problem_files_dir).mkdir(parents=True, exist_ok=True)  # create the directory

    # 
    tmp_file =  problem_files_dir + '/session.json'
    with open(tmp_file, 'w') as f:
        json.dump(dict(session),f)

    # copy the urdf and srdf files into the directory
    # write code here
    urdf_file = session.get('urdf_file')
    srdf_file = session.get('srdf_file')
    # Copy URDF file
    shutil.copyfile(urdf_file, problem_files_dir + '/urdf.urdf')
    # Copy SRDF file
    shutil.copyfile(srdf_file, problem_files_dir + '/srdf.srdf')

    # copy the meshes directory
    mesh_idr = pyrrt.DATADIR + "models/meshes"

    destination_dir = problem_files_dir + '/meshes'
    if os.path.exists(destination_dir):
        shutil.rmtree(destination_dir)

    shutil.copytree(mesh_idr,destination_dir)

    # session['urdf_file'] = pyrrt.DATADIR + data['urdf']
    # session['srdf_file'] = pyrrt.DATADIR + data['srdf']
    # session['start_vector'] = data['start']
    # session['goal_vector'] = data['goal']
    # session['lb_vector'] = data['lb']
    # session['ub_vector'] = data['ub']
    # session['idx_vis_name'] = data['idx_vis_name']
    # session["state_space"] = data['state_space']

    # Create a byte stream to hold the ZIP file
    zip_buffer = io.BytesIO()

    # with zipfile.ZipFile(zip_buffer, 'w', zipfile.ZIP_DEFLATED) as zip_file:
    #     # Walk through the directory and add files to the ZIP
    #     for root, dirs, files in os.walk(problem_files_dir):
    #         for file in files:
    #             zip_file.write(os.path.join(root, file), arcname=file)

    with zipfile.ZipFile(zip_buffer, 'w', zipfile.ZIP_DEFLATED) as zip_file:
        for root, dirs, files in os.walk(problem_files_dir):
            for file in files:
                file_path = os.path.join(root, file)
                # Determine the archive name (relative path within the ZIP file)
                arcname = os.path.relpath(file_path, start=problem_files_dir)
                zip_file.write(file_path, arcname=arcname)

    # zip_buffer.seek(0)
    # return send_file(zip_buffer, as_attachment=True, attachment_filename='problem_files.zip')




    # Set the pointer of the buffer to the beginning
    zip_buffer.seek(0)

    # Send the ZIP file to the client
    threading.Thread(target=delete_temp_files, args=(problem_files_dir,)).start()



    return send_file(zip_buffer, as_attachment=True, download_name = 'problem_files.zip')




@app.route('/upload', methods=['POST'])
def upload_file():
      # files = [request.files.get('file1'), request.files.get('file2')]
      #   for file in files:
      #       if file and file.filename:
      #           filename = secure_filename(file.filename)
      #           filepath = os.path.join(app.config['UPLOAD_FOLDER'], filename)
      #           file.save(filepath)
    urdf_file = ""
    srdf_file = ""
    file1 = request.files.get('urdf')
    if file1.filename == '':
        return redirect(request.url)
    if file1:
        filename = secure_filename(file1.filename)
        filepath = os.path.join(app.config['UPLOAD_FOLDER'], filename)
        file1.save(filepath)
        urdf_file = filepath
    file2 = request.files.get('srdf')
    if file2.filename == '':
        return redirect(request.url)
    if file2: 
        filename = secure_filename(file2.filename)
        filepath = os.path.join(app.config['UPLOAD_FOLDER'], filename)
        file2.save(filepath)
        srdf_file = filepath
    session['urdf_file'] = urdf_file
    session['srdf_file'] = srdf_file


    return render_template('my_template.html', 
                           urdf_file=session.get('urdf_file',None),
                           srdf_file=session.get('srdf_file',None),
                           start_vector=session.get('start_vector',None),
                           goal_vector=session.get('goal_vector',None))






# ([-0.62831853, 0.0, 0.0, 0.0, 0.9424778, 0.0, -0.9424778])
# ([0.62831853, 0.2, 0.3, 0.0, 0.9424778, 0.0, -0.9424778])


@app.route('/example_problem', methods=['POST'])
def example_problem_fun():
    chosen_problem = request.form['problem']

    base_path = pathlib.Path(__file__).parent.resolve()
    DD = {
        "drones_payload" :  str(base_path)+ "/example_envs/point_mass_cables.json", 
        "manip_arm" : str(base_path) + "./example_envs/ur5_bin.json",
        "two_manip_arms" : str(base_path) + "./example_envs/ur5_two_arms.json",
        "l_shape": str(base_path) + "./example_envs/se3_window.json",
    }

    json_file = DD[chosen_problem]

    #load json file
    with open(json_file) as f:
        data = json.load(f)

    # fill in the session
    session['urdf_file'] = pyrrt.DATADIR + data['urdf']
    session['srdf_file'] = pyrrt.DATADIR + data['srdf']
    session['start_vector'] = data['start']
    session['goal_vector'] = data['goal']
    session['lb_vector'] = data['lb']
    session['ub_vector'] = data['ub']
    session['idx_vis_name'] = data['idx_vis_name']
    session["state_space"] = data['state_space']
    session['chosen_problem'] = chosen_problem

    zmq_url="tcp://127.0.0.1:6000"
    viewer = meshcat.Visualizer(zmq_url)
    viewer_helper = pyrrt_vis.ViewerHelperRRT(viewer, session['urdf_file'],
                                              package_dirs=pyrrt.DATADIR + "models/meshes" ,start= np.array(session['start_vector']), goal=np.array(session['goal_vector']))
    jup = viewer.render_static()


    return render_template('my_template.html', 
                           urdf_file=session.get('urdf_file',None),
                           srdf_file=session.get('srdf_file',None),
                           start_vector=session.get('start_vector',None),
                           goal_vector=session.get('goal_vector',None),
                           additional_html=jup.data)



    return render_template('my_template.html', 
                           urdf_file=session.get('urdf_file',None),
                           srdf_file=session.get('srdf_file',None),
                           start_vector=session.get('start_vector',None),
                           goal_vector=session.get('goal_vector',None),
                           lb_vector =session.get('lb_vector',None), 
                           ub_vector = session.get('ub_vector',None), 
                           # TODO: add idx_vis_name,
                           # TODO: add state_space,
                           )




@app.route('/visualize', methods=['POST'])
def visualize():


    urdf_file = session.get('urdf_file',None)
    srdf_file = session.get('srdf_file',None)
    start = np.array(session.get('start_vector',None))
    goal = np.array(session.get('goal_vector',None))

    viewer = meshcat.Visualizer()
    viewer_helper = pyrrt_vis.ViewerHelperRRT(viewer, urdf_file, 
                                              package_dirs=pyrrt.DATADIR + "models/meshes" ,start= start, goal=goal)
    jup = viewer.render_static()

    # session['additional_html'] = 

    return render_template('my_template.html', 
                           urdf_file=session.get('urdf_file',None),
                           srdf_file=session.get('srdf_file',None),
                           start_vector=session.get('start_vector',None),
                           goal_vector=session.get('goal_vector',None),
                           additional_html=jup.data)
                           




def parse_vector(vector_string):
    return [float(x.strip()) for x in vector_string.split(',')]

@app.route('/set', methods=['POST'])
def run_set():
    try:
        # Extract and parse start and goal vectors
        
        start_vector_str = request.form['start_vector']
        goal_vector_str = request.form['goal_vector']
        start_vector = parse_vector(request.form['start_vector'])
        goal_vector = parse_vector(request.form['goal_vector'])


        # Here, implement your planning algorithm using start_vector and goal_vector
        # For demonstration, we'll just return the vectors as strings
        plan_results = f"Start: {start_vector}, Goal: {goal_vector}"

    except ValueError:
        # Handle error in case of invalid input
        plan_results = "Invalid input. Please enter vectors as comma-separated numbers."

    session['start_vector'] = start_vector
    session['goal_vector'] = goal_vector

    return render_template('my_template.html', 
                           urdf_file=session.get('urdf_file',None),
                           srdf_file=session.get('srdf_file',None),
                           start_vector=session.get('start_vector',None),
                           goal_vector=session.get('goal_vector',None))
                           # results=message, 
                           # additional_html=jup.data)


@app.route('/set_bounds', methods=['POST'])
def run_set_bounds():
    try:
        # Extract and parse start and goal vectors
        
        lb_vector_str = request.form['lb_vector']
        ub_vector_str = request.form['ub_vector']
        lb_vector = parse_vector(request.form['lb_vector'])
        ub_vector = parse_vector(request.form['ub_vector'])

    except ValueError:
        # Handle error in case of invalid input
        plan_results = "Invalid input. Please enter vectors as comma-separated numbers."

    session['lb_vector'] = lb_vector
    session['ub_vector'] = ub_vector

    return render_template('my_template.html', 
                           urdf_file=session.get('urdf_file',None),
                           srdf_file=session.get('srdf_file',None),
                           start_vector=session.get('start_vector',None),
                           goal_vector=session.get('goal_vector',None),
                           lb_vector =session.get('lb_vector',None),
                           ub_vector = session.get('ub_vector',None))


@app.route('/plan', methods=['POST'])
def run_plan():
    # ... existing code to extract and parse vectors ...

    chosen_algorithm = request.form['algorithm']
    plan_results = ""

    urdf_file = session.get('urdf_file',None)
    srdf_file = session.get('srdf_file',None)
    start = np.array(session.get('start_vector',None))
    goal = np.array(session.get('goal_vector',None))

    cm = pyrrt.Collision_manager_pinocchio()
    cm.set_urdf_filename(urdf_file)
    cm.set_srdf_filename(srdf_file)
    cm.build()

    assert cm.is_collision_free(start)
    assert cm.is_collision_free(goal)
    # mid = (goal + start) / 2.0
    # assert not cm.is_collision_free(mid)

    cm.reset_counters()
    N = 100
    for i in range(N):
        p = start + (goal - start) * i / float(N)
        col = cm.is_collision_free(p)

    # Collision Checking, based on Pinocchio and Hpp-fcl, is fast!
    message = f"Average Time of 1 collision check in [ms]: {cm.get_time_ms() / N}"
    print("message ", message)
    cm.reset_counters()

    lb = np.array(session.get('lb_vector',None))
    ub = np.array(session.get('ub_vector',None))

    viewer = meshcat.Visualizer()
    viewer_helper = pyrrt_vis.ViewerHelperRRT(viewer, urdf_file, 
                                              package_dirs=pyrrt.DATADIR + "models/meshes"
                                              , start=start, goal=goal)

    if chosen_algorithm == "RRT":



        rrt = pyrrt.PlannerRRT_Rn()
        rrt.set_start(start)
        rrt.set_goal(goal)
        rrt.init(len(start))
        rrt.set_is_collision_free_fun_from_manager(cm)
        rrt.set_bounds_to_state(lb, ub)
        print("rrt planning")
        # Create a file-like object to capture output
        output_capture = io.StringIO()

        # Redirect stdout to the file-like object
        out = rrt.plan()


        parents = rrt.get_parents()
        configs = rrt.get_configs()
        path = rrt.get_path()
        fine_path = rrt.get_fine_path(.05)

        robot = viewer_helper.robot
        viz = viewer_helper.viz

        idx_vis_name = session.get('idx_vis_name',None)
        IDX_VIS = robot.model.getFrameId(idx_vis_name)
        display_count = 0  # Just to enumerate the number
        # of objects we create in the visualizer.
        for i, p in enumerate(parents):
            if p != -1:
                q1 = configs[i]
                q2 = configs[p]
                pyrrt_vis.display_edge(robot, q1, q2, IDX_VIS, display_count, viz, radius=0.005,
                                      color=[0.2, 0.8, 0.2, 0.9])
                display_count += 1

        for i in range(len(path) - 1):
            q1 = path[i]
            q2 = path[i + 1]
            pyrrt_vis.display_edge(robot, q1, q2, IDX_VIS, display_count, viz, radius=0.02,
                                  color=[0.0, 0.0, 1.0, 0.5])
            display_count += 1

        # Finally, we can visualize the path of the robot :)
        # In standard Python, you can update the 
        # visualization with:
        # for p in fine_path:
        #     viz.display(np.array(p))
        #     time.sleep(0.01)

        # You can generate an animation as follows:
        # This is a small hack to generate a visualization -- please contact me if you know a better way!
        anim = Animation()
        __v = viewer_helper.viz.viewer
        for i in range(len(fine_path)):
            with anim.at_frame(viewer, i) as frame:
                viewer_helper.viz.viewer = frame
                viz.display(fine_path[i])

        viewer.set_animation(anim)
        viewer_helper.viz.viewer = __v
        additional_html = viewer.render_static()

        # Get the captured output
        captured_output = str(out)
        print("captured_output ", captured_output)



        # Implement RRT logic
        plan_results = "Results from RRT algorithm."
    elif chosen_algorithm == "RRTConnect":
        # Implement RRTConnect logic
        # We have implemented the most commonly used algorithms for
        # sample-based motion planning: RRT, RRT*, RRTConnect, (lazy)PRM(*)...
        # RRT_Connect is considered one of the fastest motion planners.
        # In robotic manipulation planning, because both start and goal configurations
        # are often close to the obstacles, a bidirectional search is often superior.
        rrt = pyrrt.PlannerRRTConnect_Rn()
        rrt.set_start(start)
        rrt.set_goal(goal)
        rrt.init(len(start))

        config_str = """
        [RRTConnect_options]
        max_it = 100000
        collision_resolution = 0.05
        max_step = 1.0
        max_num_configs = 100000
        """
        rrt.read_cfg_string(config_str)
        rrt.set_is_collision_free_fun_from_manager(cm)
        rrt.set_bounds_to_state(lb, ub)

        tic = time.time()
        out = rrt.plan()
        toc = time.time()
        captured_output = str(out)
        path = rrt.get_path()
        fine_path = rrt.get_fine_path(0.05)
        planner_data = rrt.get_planner_data()
        # Add a small sleep to give time for the std::cout inside the compiled library to appear on the screen
        # before we print from Python.
        time.sleep(0.001)
        print("Planning Time [s]", toc - tic)
        # We can examine the content of planner data
        print("Fields in planner_data", [i for i in planner_data])

        parents_backward = planner_data["parents_backward"]
        configs_backward = [np.array(x) for x in planner_data["configs_backward"]]
        parents = planner_data["parents"]
        configs = [np.array(x) for x in planner_data["configs"]]

        plan_results = "Results from RRTConnect algorithm."
        robot = viewer_helper.robot
        viz = viewer_helper.viz

        print("preparing to visualize")
        idx_vis_name = session['idx_vis_name']
        IDX_VIS = robot.model.getFrameId(idx_vis_name)
        display_count = 0  # just to enumerate the number

        # We display the forward and backward trees in two different colors.
        
        for i, p in enumerate(parents):
            if p != -1:
                q1 = configs[i]
                q2 = configs[p]
                # Pinocchio
                try: 
                    pyrrt_vis.display_edge(robot, q1, q2, IDX_VIS, display_count, viz, radius=0.005,
                                          color=[0.2, 0.8, 0.2, 0.9])
                except:
                    print("error in display_edge")
                display_count += 1


        for i, p in enumerate(parents_backward):
            if p != -1:
                q1 = configs_backward[i]
                q2 = configs_backward[p]
                # Pinocchio
                try:
                    pyrrt_vis.display_edge(robot, q1, q2, IDX_VIS, display_count, viz, radius=0.005,
                                          color=[0.8, 0.2, 0.2, 0.9])
                except:
                    print("error in display_edge")

                display_count += 1

        print("done")

        for i in range(len(path) - 1):
            q1 = path[i]
            q2 = path[i + 1]
            pyrrt_vis.display_edge(robot, q1, q2, IDX_VIS, display_count, viz, radius=0.02,
                                  color=[0.0, 0.0, 1.0, 0.5])
            display_count += 1

        # Finally, we can visualize the path of the robot :)
        # for p in fine_path:
        #     viz.display(np.array(p))
        #     time.sleep(0.01)

        anim = Animation()
        __v = viewer_helper.viz.viewer
        # A small hack to generate visualization. Please contact me if you know a better way!
        for i in range(len(fine_path)):
            with anim.at_frame(viewer, i) as frame:
                viewer_helper.viz.viewer = frame
                viz.display(fine_path[i])
        viewer.set_animation(anim)
        viewer_helper.viz.viewer = __v
        additional_html = viewer.render_static()

        print("done visualizing")
        # time.sleep(2)



    elif chosen_algorithm == "BiRRT":
        # Implement BiRRT logic
        plan_results = "Results from BiRRT algorithm."
    else:
        plan_results = "Invalid algorithm choice."

    return render_template('my_template.html', 
                           urdf_file=session.get('urdf_file',None),
                           srdf_file=session.get('srdf_file',None),
                           start_vector=session.get('start_vector',None),
                           goal_vector=session.get('goal_vector',None),
                           additional_html= additional_html,
                           cpp_output=captured_output)



@app.route('/download_solution_fun', methods=['POST'])
def download_solution_fun():
    print("not implemented")





@app.route('/')
def display_html():
    return render_template('my_template.html')
#     # Extract the HTML string from the IPython HTML object
#     additional_html = additional_content.data
#
#     # Render the template with the additional HTML content
#
#     # Create a new visualizer
#     vis = meshcat.Visualizer()
#
#     # By default, creating the Visualizer will start up a meshcat server for you in the background. The easiest way to open the visualizer is with its open method:
#     #
#     # vis.open()
#     #
#     # # If vis.open() does not work for you, you can also point your browser to the server's URL:
#     #
#     # vis.url()
#
#     # To create a 3D object, we use the set_object method:
#
#     vis.set_object(g.Box([0.2, 0.2, 0.2]))
#     out = vis.static_html()
#     jup = vis.jupyter_cell()
#     static = vis.render_static()
#     print(type(jup))
#     print(type(static))
#     with open('meshcat.html', 'w') as f:
#         f.write(out)
#     return render_template('my_template.html', additional_html=jup.data)

if __name__ == '__main__':
    app.run(debug=True)
