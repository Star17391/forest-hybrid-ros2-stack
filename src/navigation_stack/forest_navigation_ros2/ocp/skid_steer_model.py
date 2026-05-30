"""CasADi + acados model: unicycle / skid-steer kinematic (cmd_vel = v, omega)."""

from acados_template import AcadosModel
import casadi as ca


def export_skid_steer_kin_model() -> AcadosModel:
    model_name = "skid_steer_kin"

    px = ca.SX.sym("px")
    py = ca.SX.sym("py")
    psi = ca.SX.sym("psi")
    x = ca.vertcat(px, py, psi)

    v = ca.SX.sym("v")
    omega = ca.SX.sym("omega")
    u = ca.vertcat(v, omega)

    x_ref = ca.SX.sym("x_ref")
    y_ref = ca.SX.sym("y_ref")
    psi_ref = ca.SX.sym("psi_ref")
    v_ref = ca.SX.sym("v_ref")
    p = ca.vertcat(x_ref, y_ref, psi_ref, v_ref)

    px_dot = v * ca.cos(psi)
    py_dot = v * ca.sin(psi)
    psi_dot = omega
    f_expl = ca.vertcat(px_dot, py_dot, psi_dot)

    err_x = px - x_ref
    err_y = py - y_ref
    err_psi = ca.atan2(ca.sin(psi - psi_ref), ca.cos(psi - psi_ref))
    err_v = v - v_ref

    cost_y = ca.vertcat(err_x, err_y, err_psi, err_v, omega, v)
    cost_y_e = ca.vertcat(err_x, err_y, err_psi)

    model = AcadosModel()
    model.name = model_name
    model.x = x
    model.xdot = ca.SX.sym("xdot", x.size1())
    model.u = u
    model.p = p
    model.f_expl_expr = f_expl
    model.cost_y_expr = cost_y
    model.cost_y_expr_e = cost_y_e

    model.x_labels = ["px", "py", "psi"]
    model.u_labels = ["v", "omega"]
    model.p_labels = ["x_ref", "y_ref", "psi_ref", "v_ref"]

    return model
