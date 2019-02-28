#include "trajectory.h"


using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;




TrajectoryJMT JMT_init(double car_s, double car_d)
{
  TrajectoryJMT traj_jmt;
  // 50 x {s, s_dot, s_ddot}
  vector<PointC2> store_path_s(PARAM_NB_POINTS, PointC2(0, 0, 0));
  vector<PointC2> store_path_d(PARAM_NB_POINTS, PointC2(0, 0, 0));

  for (int i = 0; i < PARAM_NB_POINTS; i++) {
    store_path_s[i] = PointC2(car_s, 0, 0);
    store_path_d[i] = PointC2(car_d, 0, 0);
  }

  traj_jmt.path_sd.path_s = store_path_s;
  traj_jmt.path_sd.path_d = store_path_d;

  return traj_jmt;
}


Trajectory::Trajectory(std::vector<Target> targets, Map &map, CarData &car, PreviousPath &previous_path, Predictions &predictions)
{
  for (size_t i = 0; i < targets.size(); i++) {
    TrajectoryXY trajectory;
    if (PARAM_TRAJECTORY_JMT) {
      TrajectoryJMT traj_jmt;
  
      // generate JMT trajectory in s and d: converted then to (x,y) for trajectory output
        if (targets[i].time == 0)  // EMERGENCY ...
          traj_jmt = generate_trajectory_sd(targets[i], map, car, previous_path);
        else  // JMT ...
          traj_jmt = generate_trajectory_jmt(targets[i], map, previous_path);

      trajectory = traj_jmt.trajectory;
      trajectories_sd_.push_back(traj_jmt.path_sd);
    } else {
      // generate SPLINE trajectory in x and y
      trajectory = generate_trajectory(targets[i], map, car, previous_path);
    }
  
    Cost cost = Cost(trajectory, targets[i], predictions, car.lane);
    costs_.push_back(cost);
    trajectories_.push_back(trajectory);
  }
  
  // --- retrieve the lowest cost trajectory ---
  min_cost_ = INF;
  min_cost_index_ = 0;
  for (size_t i = 0; i < costs_.size(); i++) {
    if (costs_[i].get_cost() < min_cost_) {
      min_cost_ = costs_[i].get_cost();
      min_cost_index_ = i;
    }
  }

  // enforce emergency traj: last one (in case of unavoidable collision we prefer lower speed anyways)
  if (min_cost_ >= PARAM_COST_FEASIBILITY) {
    min_cost_index_ = costs_.size() - 1;
    min_cost_ = costs_[min_cost_index_].get_cost();
  }

  if (targets[min_cost_index_].time == 0) {
    car.emergency = true;
  } else {
    car.emergency = false;
  }

  if (car.emergency) {
    cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! trajectory_sd !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n";
  }
}
  

vector<double> Trajectory::JMT(vector< double> start, vector <double> end, double T)
{
    /*
    Calculate the Jerk Minimizing Trajectory that connects the initial state
    to the final state in time T.

    INPUTS

    start - the vehicles start location given as a length three array
        corresponding to initial values of [s, s_dot, s_double_dot]

    end   - the desired end state for vehicle. Like "start" this is a
        length three array.

    T     - The duration, in seconds, over which this maneuver should occur.

    OUTPUT 
    an array of length 6, each value corresponding to a coefficent in the polynomial 
    s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5

    EXAMPLE

    > JMT( [0, 10, 0], [10, 10, 0], 1)
    [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
    */

    MatrixXd A(3,3);
    VectorXd b(3);
    VectorXd x(3);

    A <<   pow(T,3),    pow(T,4),    pow(T,5),
         3*pow(T,2),  4*pow(T,3),  5*pow(T,4),
                6*T, 12*pow(T,2), 20*pow(T,3);

    b << end[0] - (start[0] + start[1]*T + 0.5*start[2]*T*T), 
         end[1] - (start[1] + start[2]*T), 
         end[2] - start[2];

    x = A.inverse() * b;

    return {start[0], start[1], start[2]/2, x[0], x[1], x[2]};
}

// c: coefficients of polynom
double Trajectory::polyeval(vector<double> c, double t) {
  double res = 0.0;
  for (size_t i = 0; i < c.size(); i++) {
    res += c[i] * pow(t, i);
  }
  return res;
}

// 1st derivative of a polynom
double Trajectory::polyeval_dot(vector<double> c, double t) {
  double res = 0.0;
  for (size_t i = 1; i < c.size(); ++i) {
    res += i * c[i] * pow(t, i-1);
  }
  return res;
}

// 2nd derivative of a polynom
double Trajectory::polyeval_ddot(vector<double> c, double t) {
  double res = 0.0;
  for (size_t i = 2; i < c.size(); ++i) {
    res += i * (i-1) * c[i] * pow(t, i-2);
  }
  return res;
}



TrajectoryJMT Trajectory::generate_trajectory_jmt(Target target, Map &map, PreviousPath const &previous_path)
{
  TrajectoryJMT traj_jmt;

  TrajectoryXY previous_path_xy = previous_path.xy;
  int prev_size = previous_path.num_xy_reused;
  TrajectorySD prev_path_sd = previous_path.sd;

  vector<double> previous_path_x = previous_path_xy.x_vals;
  vector<double> previous_path_y = previous_path_xy.y_vals;
  vector<PointC2> prev_path_s = prev_path_sd.path_s;
  vector<PointC2> prev_path_d = prev_path_sd.path_d;

  vector<PointC2> new_path_s(PARAM_NB_POINTS, PointC2(0,0,0));
  vector<PointC2> new_path_d(PARAM_NB_POINTS, PointC2(0,0,0));

  //cout << "prev_size=" << prev_size << endl;
  //int last_point = PARAM_NB_POINTS - prev_size - 1;
  int last_point;
  if (PARAM_PREV_PATH_XY_REUSED < PARAM_NB_POINTS) {
    last_point = PARAM_NB_POINTS - previous_path_x.size() + prev_size - 1;
  } else {
    last_point = PARAM_NB_POINTS - 1;
  }

  double T = target.time; // 2 seconds si car_d center of line

  double si, si_dot=0, si_ddot;
  double di, di_dot, di_ddot;

  si      = prev_path_s[last_point].f;
  si_dot  = prev_path_s[last_point].f_dot;
  si_ddot = prev_path_s[last_point].f_ddot;

  di      = prev_path_d[last_point].f;
  di_dot  = prev_path_d[last_point].f_dot;
  di_ddot = prev_path_d[last_point].f_ddot;

  double sf, sf_dot, sf_ddot;
  double df, df_dot, df_ddot;

  if (target.velocity <= 10) { // mph
    df_ddot =  0;
    df_dot  =  0;
    df      = di;

    sf_ddot = 0;
    sf_dot  = mph_to_ms(target.velocity);

    sf      = si + 2 * sf_dot * T;
  } else {
    df_ddot = 0;
    df_dot  = 0;
    df      = get_dcenter(target.lane);

    sf_ddot = 0;
    sf_dot = mph_to_ms(target.velocity);
    // we use JMT for lane changes only
    // no need to try to reach max speed during lane changes
    sf_dot = min(sf_dot, 0.9 * PARAM_MAX_SPEED);

    sf = si + sf_dot * T;
  }

  vector<double> start_s = { si, si_dot, si_ddot};
  vector<double> end_s = { sf, sf_dot, 0};

  vector<double> start_d = { di, di_dot, di_ddot };
  vector<double> end_d = { df, df_dot, df_ddot};

  /////////////////////////////////////////////////////////////

  vector<double> poly_s = JMT(start_s, end_s, T);
  vector<double> poly_d = JMT(start_d, end_d, T);

  polysJmt_.push_back({ poly_s, poly_d, target.time });

  vector<double> next_x_vals;
  vector<double> next_y_vals;
  
  for (int i = 0; i < prev_size; i++) {
    new_path_s[i] = prev_path_s[PARAM_NB_POINTS - previous_path_x.size() + i];
    new_path_d[i] = prev_path_d[PARAM_NB_POINTS - previous_path_x.size() + i];

    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
  }

  //double t = 0.0; continuity point reused
  double t = PARAM_DT;
  for (int i = prev_size; i < PARAM_NB_POINTS; i++) {
    double s = polyeval(poly_s, t);
    double s_dot = polyeval_dot(poly_s, t);
    double s_ddot = polyeval_ddot(poly_s, t);

    double d = polyeval(poly_d, t);
    double d_dot = polyeval_dot(poly_d, t);
    double d_ddot = polyeval_ddot(poly_d, t);

    new_path_s[i] = PointC2(s, s_dot, s_ddot);
    new_path_d[i] = PointC2(d, d_dot, d_ddot);

    vector<double> point_xy = map.getXYspline(s, d);

    next_x_vals.push_back(point_xy[0]);
    next_y_vals.push_back(point_xy[1]);

    t += PARAM_DT;
  }

  traj_jmt.trajectory = TrajectoryXY(next_x_vals, next_y_vals);
  traj_jmt.path_sd = TrajectorySD(new_path_s, new_path_d);

  return traj_jmt;
}


// trajectory generated in (s, d) Frenet coordinates 
// - with constant accel/decel (no JMT here) in between 2 s waypoints
// - without d changes (we stay in the same lane)
TrajectoryJMT Trajectory::generate_trajectory_sd(Target target, Map &map, CarData const &car, PreviousPath const &previous_path)
{
  TrajectoryJMT traj_jmt;

  TrajectoryXY previous_path_xy = previous_path.xy;
  int prev_size = previous_path.num_xy_reused;
  TrajectorySD prev_path_sd = previous_path.sd;

  vector<double> previous_path_x = previous_path_xy.x_vals;
  vector<double> previous_path_y = previous_path_xy.y_vals;
  vector<PointC2> prev_path_s = prev_path_sd.path_s;
  vector<PointC2> prev_path_d = prev_path_sd.path_d;

  vector<PointC2> new_path_s(PARAM_NB_POINTS, PointC2(0,0,0));
  vector<PointC2> new_path_d(PARAM_NB_POINTS, PointC2(0,0,0));

  vector<double> next_x_vals;
  vector<double> next_y_vals;

  double target_velocity_ms = mph_to_ms(target.velocity);

  double s, s_dot, s_ddot;
  double d, d_dot, d_ddot;
  if (prev_size > 0) {
    for (int i = 0; i < prev_size; i++) {
      new_path_s[i] = prev_path_s[PARAM_NB_POINTS - previous_path_x.size() + i];
      new_path_d[i] = prev_path_d[PARAM_NB_POINTS - previous_path_x.size() + i];

      next_x_vals.push_back(previous_path_x[i]);
      next_y_vals.push_back(previous_path_y[i]);
    }

    // initial conditions for new (s,d) trajectory
    s = new_path_s[prev_size-1].f;
    s_dot = new_path_s[prev_size-1].f_dot;
    d = new_path_d[prev_size-1].f, d_dot = 0, d_ddot = 0;
  } else {
    s = car.s, s_dot = car.speed;
    d = car.d, d_dot = 0, d_ddot = 0;
  }

  s_ddot = target.accel;  //-PARAM_MAX_ACCEL;

  //double t = 0.0; continuity point reused
  double t = PARAM_DT;
  double prev_s_dot = s_dot;
  for (int i = prev_size; i < PARAM_NB_POINTS; i++) {

    // increase/decrease speed till target velocity is reached
    s_dot += s_ddot * PARAM_DT; 
    if ((target.accel > 0 && prev_s_dot <= target_velocity_ms && s_dot > target_velocity_ms) ||
        (target.accel < 0 && prev_s_dot >= target_velocity_ms && s_dot < target_velocity_ms)) {
      s_dot = target_velocity_ms;
    }
    s_dot = max(min(s_dot, 0.9 * PARAM_MAX_SPEED), 0.0);
    s += s_dot * PARAM_DT;

    prev_s_dot = s_dot;

    new_path_s[i] = PointC2(s, s_dot, s_ddot);
    new_path_d[i] = PointC2(d, d_dot, d_ddot);

    vector<double> point_xy = map.getXYspline(s, d);

    next_x_vals.push_back(point_xy[0]);
    next_y_vals.push_back(point_xy[1]);

    t += PARAM_DT;
  }

  traj_jmt.trajectory = TrajectoryXY(next_x_vals, next_y_vals);
  traj_jmt.path_sd = TrajectorySD(new_path_s, new_path_d);

  return traj_jmt;
}


TrajectoryXY Trajectory::generate_trajectory(Target target, Map &map, CarData const &car, PreviousPath const &previous_path)
{
  TrajectoryXY previous_path_xy = previous_path.xy;
  int prev_size = previous_path.num_xy_reused;

  vector<double> previous_path_x = previous_path_xy.x_vals;
  vector<double> previous_path_y = previous_path_xy.y_vals;

  vector<double> ptsx;
  vector<double> ptsy;
  
  double ref_x = car.x;
  double ref_y = car.y;
  double ref_yaw = deg2rad(car.yaw);

  if (prev_size < 2) {
    double prev_car_x = car.x - cos(car.yaw);
    double prev_car_y = car.y - sin(car.yaw);
  
    ptsx.push_back(prev_car_x);
    ptsx.push_back(car.x);
  
    ptsy.push_back(prev_car_y);
    ptsy.push_back(car.y);
  } else {
    ref_x = previous_path_x[prev_size-1];
    ref_y = previous_path_y[prev_size-1];
  
    double ref_x_prev = previous_path_x[prev_size-2];
    double ref_y_prev = previous_path_y[prev_size-2];
    ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);
  
    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);
  
    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);
  }
  
  //vector<double> next_wp0 = map.getXY(car.s+30, get_dcenter(target.lane));
  //vector<double> next_wp1 = map.getXY(car.s+60, get_dcenter(target.lane));
  //vector<double> next_wp2 = map.getXY(car.s+90, get_dcenter(target.lane));

  vector<double> next_wp0 = map.getXYspline(car.s+30, get_dcenter(target.lane));
  vector<double> next_wp1 = map.getXYspline(car.s+60, get_dcenter(target.lane));
  vector<double> next_wp2 = map.getXYspline(car.s+90, get_dcenter(target.lane));
  
  
  ptsx.push_back(next_wp0[0]);
  ptsx.push_back(next_wp1[0]);
  ptsx.push_back(next_wp2[0]);
  
  ptsy.push_back(next_wp0[1]);
  ptsy.push_back(next_wp1[1]);
  ptsy.push_back(next_wp2[1]);
  
  
  for (size_t i = 0; i < ptsx.size(); i++) {
    // shift car reference angle to 0 degrees
    // transformation to local car's coordinates (cf MPC)
    // last point of previous path at origin and its angle at zero degree
  
    // shift and rotation
    double shift_x = ptsx[i]-ref_x;
    double shift_y = ptsy[i]-ref_y;
  
    ptsx[i] = (shift_x * cos(0-ref_yaw) - shift_y * sin(0 - ref_yaw));
    ptsy[i] = (shift_x * sin(0-ref_yaw) + shift_y * cos(0 - ref_yaw));
  }
  
  
  tk::spline spl;
  spl.set_points(ptsx, ptsy);
  
  vector<double> next_x_vals;
  vector<double> next_y_vals;
  
  for (int i = 0; i < prev_size; i++) {
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
  }
  
  // Calculate how to break up spline points so that we travel at our desired reference velocity
  double target_x = 30.0;
  double target_y = spl(target_x);
  double target_dist = sqrt(target_x*target_x + target_y*target_y);
  
  double x_add_on = 0;
  
  // fill up the rest of our path planner after filing it with previous points
  // here we will always output 50 points
  for (int i = 1; i <= PARAM_NB_POINTS - prev_size; i++) {
    double N = (target_dist / (PARAM_DT * mph_to_ms(target.velocity))); // divide by 2.24: mph -> m/s
    double x_point = x_add_on + target_x/N;
    double y_point = spl(x_point);
  
    x_add_on = x_point;
  
    double x_ref = x_point; // x_ref IS NOT ref_x !!!
    double y_ref = y_point;
  
    // rotate back to normal after rotating it earlier
    x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
    y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));
  
    x_point += ref_x;
    y_point += ref_y;
  
    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);
  }

  //return { next_x_vals, next_y_vals };
  return TrajectoryXY(next_x_vals, next_y_vals);
}
