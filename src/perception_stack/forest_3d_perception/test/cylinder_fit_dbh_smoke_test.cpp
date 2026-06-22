// Smoke test do DBH robusto da perceção (Agente 2), sem ROS/sim:
//   [1] Kåsa circle fit recupera centro/raio de um círculo completo.
//   [2] Arco parcial (oclusão LiDAR): a média enviesa o centro e subestima o raio;
//       o Kåsa recupera ambos — justifica o circle fit no cylinder_fit.hpp.
//   [3] Tronco inclinado em base_link: sem gravity-align o DBH infla; com align
//       (Rx(-roll)Ry(-pitch) antes do fit) recupera o DBH real.
// Exit 0 = todas passaram. Corre via `colcon test --packages-select forest_3d_perception`.
#include <cmath>
#include <cstdio>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "forest_3d_perception/cylinder_fit.hpp"

using namespace forest_3d_perception;

static void gravity_rotation(double roll, double pitch, double R[3][3]) {
  const double cr = std::cos(-roll), sr = std::sin(-roll);
  const double cp = std::cos(-pitch), sp = std::sin(-pitch);
  R[0][0]=cp; R[0][1]=0; R[0][2]=sp;
  R[1][0]=sr*sp; R[1][1]=cr; R[1][2]=-sr*cp;
  R[2][0]=-cr*sp; R[2][1]=sr; R[2][2]=cr*cp;
}

int main() {
  int fails = 0;

  // 1. Círculo completo: centro (3,1), raio 0.15.
  {
    std::vector<float> xs, ys;
    for (int i = 0; i < 32; ++i) {
      double a = 2*M_PI*i/32;
      xs.push_back(3.0 + 0.15*std::cos(a));
      ys.push_back(1.0 + 0.15*std::sin(a));
    }
    double cx, cy, r;
    bool ok = fit_circle_kasa(xs, ys, cx, cy, r);
    printf("[1] full circle: ok=%d cx=%.4f cy=%.4f r=%.4f (esperado 3,1,0.15)\n", ok, cx, cy, r);
    if (!ok || std::abs(cx-3)>1e-3 || std::abs(cy-1)>1e-3 || std::abs(r-0.15)>1e-3) { printf("   FALHOU\n"); ++fails; }
  }

  // 2. Arco PARCIAL (120°, lado virado ao sensor na origem). Média enviesa; Kåsa não.
  {
    std::vector<float> xs, ys;
    // tronco em (3,0) raio 0.15; LiDAR na origem vê o arco virado para -x (frente)
    for (int i = 0; i < 13; ++i) {
      double a = M_PI - M_PI/3 + (2*M_PI/3)*i/12;  // arco de 120° à volta de a=180°
      xs.push_back(3.0 + 0.15*std::cos(a));
      ys.push_back(0.0 + 0.15*std::sin(a));
    }
    // média
    double mx=0,my=0; for (size_t i=0;i<xs.size();++i){mx+=xs[i];my+=ys[i];}
    mx/=xs.size(); my/=ys.size();
    double mean_r=0; for(size_t i=0;i<xs.size();++i) mean_r+=std::hypot(xs[i]-mx,ys[i]-my);
    mean_r/=xs.size();
    double cx, cy, r;
    bool ok = fit_circle_kasa(xs, ys, cx, cy, r);
    printf("[2] partial arc 120°: MEAN cx=%.4f (vies p/ sensor) | KASA ok=%d cx=%.4f cy=%.4f r=%.4f (esperado 3,0,0.15)\n",
           mx, ok, cx, cy, r);
    printf("    raio MEAN=%.4f (subestima)  raio KASA=%.4f\n", mean_r, r);
    if (!ok || std::abs(cx-3)>5e-3 || std::abs(cy-0)>5e-3 || std::abs(r-0.15)>5e-3) { printf("   FALHOU\n"); ++fails; }
    if (std::abs(mx-3) < std::abs(cx-3)) { printf("   AVISO: média não estava mais enviesada que Kåsa\n"); }
  }

  // 3. Cilindro inclinado em base_link: gerar tronco vertical (gravidade), rodar por
  //    roll=10° pitch=8°, depois gravity-align e fit. DBH deve recuperar.
  {
    double roll = 10*M_PI/180, pitch = 8*M_PI/180;
    // tronco vertical real: centro xy (4,2), raio 0.12, z de 0 a 2.0
    pcl::PointCloud<pcl::PointXYZ> tilted;
    for (int zi = 0; zi < 20; ++zi) {
      double z = 0.1*zi;
      for (int i = 0; i < 16; ++i) {
        double a = 2*M_PI*i/16;
        double x = 4.0 + 0.12*std::cos(a);
        double y = 2.0 + 0.12*std::sin(a);
        // rodar por R_tilt = Ry(pitch)Rx(roll) para simular base_link inclinado
        // (inversa do gravity align). Aqui aplico a inversa de R_align.
        double Rinv[3][3]; gravity_rotation(-roll, -pitch, Rinv); // R_align(-) = R_tilt
        pcl::PointXYZ p;
        p.x = Rinv[0][0]*x+Rinv[0][1]*y+Rinv[0][2]*z;
        p.y = Rinv[1][0]*x+Rinv[1][1]*y+Rinv[1][2]*z;
        p.z = Rinv[2][0]*x+Rinv[2][1]*y+Rinv[2][2]*z;
        tilted.push_back(p);
      }
    }
    std::vector<std::size_t> idx(tilted.size());
    for (size_t i=0;i<idx.size();++i) idx[i]=i;

    // Fit SEM align (na nuvem inclinada) — DBH inflacionado.
    CylinderObservation c_no;
    fit_vertical_cylinder(tilted, idx, c_no, 0.3, 0.8, 0.10, 0.2, 0.05, 2.5);
    // Fit COM align.
    double R[3][3]; gravity_rotation(roll, pitch, R);
    pcl::PointCloud<pcl::PointXYZ> level;
    for (auto& p : tilted.points) {
      pcl::PointXYZ q;
      q.x=R[0][0]*p.x+R[0][1]*p.y+R[0][2]*p.z;
      q.y=R[1][0]*p.x+R[1][1]*p.y+R[1][2]*p.z;
      q.z=R[2][0]*p.x+R[2][1]*p.y+R[2][2]*p.z;
      level.push_back(q);
    }
    CylinderObservation c_al;
    auto rej = fit_vertical_cylinder(level, idx, c_al, 0.3, 0.8, 0.10, 0.2, 0.05, 2.5);
    printf("[3] tronco inclinado (roll10 pitch8) raio real 0.12:\n");
    printf("    SEM align: raio=%.4f (DBH=%.3f) rmse=%.4f\n", c_no.radius, 2*c_no.radius, c_no.rmse);
    printf("    COM align: raio=%.4f (DBH=%.3f) rmse=%.4f cx=%.3f cy=%.3f (esperado 4,2)\n",
           c_al.radius, 2*c_al.radius, c_al.rmse, c_al.cx, c_al.cy);
    if (rej != CylinderReject::Accepted || std::abs(c_al.radius-0.12)>0.01) { printf("   FALHOU align\n"); ++fails; }
    if (c_no.radius <= c_al.radius) { printf("   AVISO: sem-align não inflacionou o raio como esperado\n"); }
  }

  // 4. Tronco FINO (raio 0.12, z 0–2.0) + COPA LARGA (raio 0.6, z 2.0–4.0), vistos
  //    só do lado do sensor (semicírculos, oclusão real). O fit stem-aware deve
  //    seguir o tronco e CORTAR a copa → DBH ~0.24, não inflado pela copa.
  {
    pcl::PointCloud<pcl::PointXYZ> tree;
    // tronco: raio 0.12, centro (5,0)
    for (int zi = 0; zi < 20; ++zi) {
      double z = 0.1 * zi;
      for (int i = 0; i < 10; ++i) {            // só meio arco (lado -x, virado à origem)
        double a = M_PI - M_PI/2 + (M_PI) * i/9;
        tree.push_back({(float)(5.0 + 0.12*std::cos(a)), (float)(0.12*std::sin(a)), (float)z});
      }
    }
    // copa: raio 0.6, centro (5,0), z 2.0–4.0
    for (int zi = 0; zi < 20; ++zi) {
      double z = 2.0 + 0.1 * zi;
      for (int i = 0; i < 14; ++i) {
        double a = M_PI - M_PI/2 + (M_PI) * i/13;
        tree.push_back({(float)(5.0 + 0.6*std::cos(a)), (float)(0.6*std::sin(a)), (float)z});
      }
    }
    std::vector<std::size_t> idx(tree.size());
    for (size_t i=0;i<idx.size();++i) idx[i]=i;
    CylinderObservation c;
    auto rej = fit_vertical_cylinder(tree, idx, c, 0.3, 0.8, 0.10, 0.2, 0.05, 4.0);
    // Comparar com banda fixa cega que apanharia a copa (high=4.0, grow enorme).
    CylinderObservation c_blind;
    fit_vertical_cylinder(tree, idx, c_blind, 0.3, 0.8, 0.10, 0.2, 0.05, 4.0, 0.3, 4.0, 100.0);
    printf("[4] tronco fino 0.12 + copa larga 0.6 (semicírculos):\n");
    printf("    STEM-AWARE: raio=%.4f (DBH=%.3f)  vs  banda-cega: raio=%.4f (DBH=%.3f)\n",
           c.radius, 2*c.radius, c_blind.radius, 2*c_blind.radius);
    if (rej != CylinderReject::Accepted || std::abs(c.radius-0.12)>0.04) {
      printf("   FALHOU: stem-aware não recuperou o raio do tronco (~0.12)\n"); ++fails;
    }
    if (c_blind.radius <= c.radius) {
      printf("   AVISO: banda-cega não inflou como esperado\n");
    }
  }

  // 5. Arco PARCIAL pequeno (~80°, raio 0.13): o Kåsa algébrico infla o raio; o
  //    refinamento geométrico de Landau deve corrigir para perto de 0.13.
  {
    std::vector<float> xs, ys;
    for (int i = 0; i < 10; ++i) {
      double a = M_PI - 0.7 + (1.4) * i/9;   // arco de ~80° centrado em 180°
      xs.push_back(5.0 + 0.13*std::cos(a));
      ys.push_back(0.0 + 0.13*std::sin(a));
    }
    double cx, cy, r;
    fit_circle_kasa(xs, ys, cx, cy, r);
    double rk = r;
    double cx2=cx, cy2=cy, r2=r;
    refine_circle_landau(xs, ys, cx2, cy2, r2);
    printf("[5] arco parcial ~80° raio 0.13: KASA r=%.3f -> LANDAU r=%.3f\n", rk, r2);
    if (std::abs(r2-0.13) > 0.03) { printf("   FALHOU: Landau não recuperou ~0.13\n"); ++fails; }
    if (rk <= r2 && std::abs(rk-0.13) > std::abs(r2-0.13)) { /* ok */ }
  }

  // 6. PASSO 1 (base consistente): um fit REJEITADO pela qualidade (max_rmse
  //    impossível) deve mesmo assim devolver o centro/raio da BANDA (não zeros nem
  //    o centróide enviesado da nuvem inteira). Isto garante que o consumidor usa
  //    sempre a mesma definição de base → sem teleporte accept<->fallback.
  {
    pcl::PointCloud<pcl::PointXYZ> trunk;
    for (int zi = 0; zi < 20; ++zi) {
      double z = 0.1 * zi;
      for (int i = 0; i < 12; ++i) {           // semicírculo virado ao sensor
        double a = M_PI - M_PI/2 + (M_PI) * i/11;
        // ruído radial determinístico (~±2cm) → rmse não-nulo, dispara o gate
        double rn = 0.13 + 0.02 * (((zi + i) % 3) - 1);
        trunk.push_back({(float)(6.0 + rn*std::cos(a)), (float)(rn*std::sin(a)), (float)z});
      }
    }
    std::vector<std::size_t> idx(trunk.size());
    for (size_t i=0;i<idx.size();++i) idx[i]=i;
    CylinderObservation c;
    // max_rmse_m = 1e-5 → impossível de passar → HighRmse, mas out preenchido.
    auto rej = fit_vertical_cylinder(trunk, idx, c, 0.3, 0.8, 1e-5, 0.2, 0.05, 2.5);
    printf("[6] fit rejeitado (rmse gate impossível): rej=%d valid=%d cx=%.3f cy=%.3f r=%.3f (esperado 6,0,0.13)\n",
           (int)rej, (int)c.valid, c.cx, c.cy, c.radius);
    if (rej == CylinderReject::Accepted) { printf("   FALHOU: devia rejeitar\n"); ++fails; }
    if (c.valid) { printf("   FALHOU: valid devia ser false\n"); ++fails; }
    if (std::abs(c.cx-6.0)>0.03 || std::abs(c.cy-0.0)>0.03 || std::abs(c.radius-0.13)>0.03) {
      printf("   FALHOU: base/raio da banda não preenchidos no reject (Causa A não corrigida)\n"); ++fails;
    }
  }

  // 7. COBERTURA ANGULAR (incerteza honesta do DBH): um arco LARGO deve dar
  //    arc_coverage MAIOR que um arco CURTO do mesmo tronco. É o sinal que o node
  //    usa para inflar diameter_stddev em arcos parciais (o rmse não o capta).
  {
    auto make_arc = [](double arc_deg, pcl::PointCloud<pcl::PointXYZ>& cl){
      for (int zi=0; zi<20; ++zi){ double z=0.1*zi;
        for (int i=0;i<12;++i){ double half=arc_deg*M_PI/180.0/2.0;
          double a = M_PI - half + (2*half)*i/11;
          cl.push_back({(float)(7.0+0.2*std::cos(a)),(float)(0.2*std::sin(a)),(float)z}); } }
    };
    pcl::PointCloud<pcl::PointXYZ> wide, narrow;
    make_arc(170, wide); make_arc(60, narrow);
    std::vector<std::size_t> iw(wide.size()), in(narrow.size());
    for(size_t i=0;i<iw.size();++i) iw[i]=i; for(size_t i=0;i<in.size();++i) in[i]=i;
    CylinderObservation cw, cn;
    fit_vertical_cylinder(wide, iw, cw, 0.3, 0.8, 0.10, 0.2, 0.05, 2.5);
    fit_vertical_cylinder(narrow, in, cn, 0.3, 0.8, 0.10, 0.2, 0.05, 2.5);
    printf("[7] cobertura: arco170°=%.3f  arco60°=%.3f (largo deve ser > curto)\n",
           cw.arc_coverage, cn.arc_coverage);
    if (!(cw.arc_coverage > cn.arc_coverage + 0.03f)) {
      printf("   FALHOU: cobertura não distingue arco largo de curto\n"); ++fails;
    }
  }

  // 8. SEMENTE TAUBIN vs KÅSA (viés do raio em arco parcial): num arco curto a
  //    semente algébrica Kåsa enviesa o raio (~-13% medido); o Taubin é quase sem
  //    viés. Ambas alimentam o refinamento Landau, mas uma semente melhor dá um DBH
  //    melhor. Assertiva: no arco de 60° o erro do Taubin < erro do Kåsa.
  {
    std::vector<float> xs, ys;
    const int N = 40; const double half = 60.0 * M_PI / 180.0 / 2.0;
    for (int i = 0; i < N; ++i) {
      const double a = M_PI - half + (2 * half) * i / (N - 1);
      const double rr = 0.15 + 0.003 * (((i) % 3) - 1);  // ruído radial determinístico
      xs.push_back(static_cast<float>(3.0 + rr * std::cos(a)));
      ys.push_back(static_cast<float>(0.0 + rr * std::sin(a)));
    }
    double kx, ky, kr, tx, ty, tr;
    const bool ok_k = fit_circle_kasa(xs, ys, kx, ky, kr);
    const bool ok_t = fit_circle_taubin(xs, ys, tx, ty, tr);
    const double err_k = std::abs(kr - 0.15), err_t = std::abs(tr - 0.15);
    printf("[8] arco 60°: Kasa r=%.4f (err %.4f)  Taubin r=%.4f (err %.4f) — Taubin deve ser menor\n",
           kr, err_k, tr, err_t);
    if (!ok_k || !ok_t || !(err_t < err_k)) {
      printf("   FALHOU: semente Taubin não reduziu o viés do raio em arco parcial\n"); ++fails;
    }
  }

  printf("\n%s (%d falhas)\n", fails==0?"TODOS PASSARAM":"HOUVE FALHAS", fails);
  return fails;
}
