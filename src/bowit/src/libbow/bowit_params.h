//
// Created by airlab on 11/23/23.
//
#pragma once 

#include <iostream>
#include <thread>
#include <future>
#define USE_NLOPT

#include <limbo/limbo.hpp>
#include <limbo/experimental/acqui/eci.hpp>
#include <limbo/experimental/bayes_opt/cboptimizer.hpp>

/*
add these lines to cpp_propoerties.json in vscode 
        "/home/airlab/bowit_ws/build/bowit/_deps/limbo-src/src/**",
        "/home/airlab/bowit_ws/build/bowit/install/include/**",
*/

using namespace limbo;

struct Params {
    struct bayes_opt_cboptimizer : public defaults::bayes_opt_cboptimizer {
    };

    struct init_randomsampling {
        BO_PARAM(int, samples, 40);
    };

    struct init_gridsampling{
        BO_PARAM(int, bins, 15);
    };

    struct kernel : public defaults::kernel {
        BO_PARAM(double, noise, 0.0);
    };

    struct kernel_exp : public defaults::kernel_exp {
    };

    struct bayes_opt_bobase : public defaults::bayes_opt_bobase {
        BO_PARAM(bool, stats_enabled, false);
    };

    struct stop_maxiterations {
        BO_PARAM(int, iterations, 1);
    };

    struct acqui_eci : public defaults::acqui_eci {
    };

    struct mean_constant {
        BO_PARAM(double, constant, 1.0);
    };

#ifdef USE_NLOPT
    struct opt_nloptnograd : public defaults::opt_nloptnograd {
    };
#elif defined(USE_LIBCMAES)
    struct opt_cmaes : public defaults::opt_cmaes {
    };
#else
    struct opt_gridsearch : public defaults::opt_gridsearch {
    };
#endif
};


struct GPParams {
    struct kernel_exp {
        BO_PARAM(double, sigma_sq, 1.0);
        BO_PARAM(double, l, 0.2);
    };
    struct kernel : public defaults::kernel {
    };
    struct kernel_squared_exp_ard : public defaults::kernel_squared_exp_ard {
    };

    struct kernel_maternfivehalves : public defaults::kernel_maternfivehalves {
    };
    struct opt_rprop : public defaults::opt_rprop {
    };
};

