# See LICENSE.TT for license details.
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import math
from tqdm import tqdm

def dataflow_model(databits, t_mem, M,N,K, l2_cache, kl, vlB, mlB, num_mregs, t_op_ind, width_mmu):
    """
    From 
        databits: number of bits per vector element 
        t_mem: memory latency,  
        'l2_size': cache size in KB,
        kl: number of outer product operations accumulated per instruction,
        vlB, mlB: bytes per vector, vectors per matrix register
        num_regs: number of 2D matrix registers
        t_op_ind: select functional unit latency
        width_mmu: half width reduces bw and increases latency both by a factor of four
    Calculate 
        'mem_bw': average memory bandwidth,
        'max_mem_bw': peak memory bandwidth,
        't_uk': ukernel latency,
        'ops_cycle': macc operations per cycle,
        'insts_cycle': instructions per cycle,
        'mrf_bw': matrix register file bandwidth
        'mrf_capacity': matrix register file capacity
    """
    # calculate number of parallel memory requests to hide latency
    mlf = mlB/(databits/8) #num MMU rows equals number of elements ml
    ml = min(M, mlf)
    vlf = vlB/(databits/8)
    vl = min(N, vlf)
    kl = min(K, kl)
    # CACHE
    # double buffer B[kl * vlB] and C[ml * vlB]*nregs
    # a = mlB*(kc+kl) * num_mregs
    c = ml*vlB 
    mc = min(M, num_mregs * ml)
    l2_cache_B = l2_cache*2**10
    kc = (l2_cache_B - 2*c)/(mc * databits + vlB) - kl
    kc = min(kc, K)
    l3_size = N*kc*databits/2**23 #[MB]

    #different opacc fu latencies
    t_op = [
        2*ml + kc,
        ml + kc,
        max(ml, kc)
    ]
    t_crit = t_op[t_op_ind]/width_mmu**2
    t_uk = 2*t_mem + (2*ml + kc)/width_mmu**2
    t_eff_opacc = max(t_uk, num_mregs*t_crit)

    insts_cycle = (kc/kl)/t_eff_opacc

    util_t = min(1, num_mregs*kc/t_uk)
    util_a = M*N / (ml*math.ceil(M/ml) * vl*math.ceil(N/vl))
    util = util_t*util_a
    ops_cycle = util*ml*vl*(databits/8)

    p_l2_op = t_uk/t_crit
    max_mregs = math.ceil(p_l2_op)
    max_mrf_capacity = max_mregs*(ml*vlB + mlB*kl + vlB*kl)/2**10
    #TODO: p_l3_l2 = t_l2*bw_l2

    a_mem = num_mregs*mc*kc*databits/8
    b_mem = kc*vlB
    c_mem = num_mregs * ml*vlB
    iterM = M/mc
    iterN = N/vl
    # iterK = K/kc
    t_blas = t_eff_opacc*iterM*iterN
    mem_bw = (c_mem*iterM*iterN + a_mem*iterM + b_mem)/t_blas
    max_mem_bw = (ml*vlB + kc*mlB + kc*vlB)/t_crit
    
    # M REGFILE
    mrf_capacity = num_mregs*(ml*vlB + mlB*kl + vlB*kl)
    mrf_bw = ml*vlB/t_crit + mlB + vlB

    #macc cell area in units of 1b adders
    adder_cell = 20 #cmos gates
    macc_cell = adder_cell*databits**2
    macc_gates = vl*ml*macc_cell
    reg_cell = 20 #cmos gates
    mrf_gates = mrf_capacity*8*reg_cell
    opu_gates = mrf_gates + macc_gates

    perf_specs = {
        't_uk': t_uk,
        # 'util_a': util_a,
        # 'util_t': util_t,
        'util': util,
        'ops_cycle': ops_cycle,
        'insts_cycle': insts_cycle,

        'max_mregs': max_mregs,
        'mrf_capacity': mrf_capacity,
        'max_mrf_capacity': max_mrf_capacity,
        # 'l3_size': l3_size,
        'mem_bw': mem_bw,
        'max_mem_bw': max_mem_bw,
        'mrf_capacity': mrf_capacity/2**10, # [kB]
        'mrf_bw': mrf_bw,

        'mrf_gates': mrf_gates,
        'macc_gates': macc_gates,
        'opu_gates': opu_gates
    }
    return perf_specs


# def generate_df(databits, M,N,K, mlB,vlB,kl, t_mem, flow_key):
def generate_df(databits, t_mem, M,N,K, l2_cache, kl, vlB, mlB, num_mregs, t_op, width_mmu):
    # Create the df index space
    index_space = [databits, t_mem, M, N, K, l2_cache, kl, vlB, mlB, num_mregs, t_op, width_mmu]
    index_labels = ['databits', 't_mem', 'M','N','K', 'l2_cache', 'kl', 'vlB', 'mlB', 'num_mregs', 't_op', 'width_mmu']
    # define df index over all possible combinations of input elements (cross product)
    df_index = pd.MultiIndex.from_product(index_space, names=index_labels)
    # Create columns of  DataFrame 
    df_columns = ['t_uk', 'util',
                  'ops_cycle', 'max_mregs', 'max_mrf_capacity',
                  'mem_bw', 'max_mem_bw',
                  'mrf_capacity', 'mrf_bw', 
                  'macc_gates', 'mrf_gates', 'opu_gates', 
                  'insts_cycle']
    df = pd.DataFrame(index=df_index, columns=df_columns,dtype=float)

    #compute performance specs
    # idxs = pd.MultiIndex.from_product(index_space, names=index_labels)
    for idx in tqdm(df_index):
        perf_specs = dataflow_model(*idx)
        df.loc[idx, 't_uk'] = perf_specs['t_uk']
        # df.loc[idx, 'util_a'] = perf_specs['util_a']
        # df.loc[idx, 'util_t'] = perf_specs['util_t']
        df.loc[idx, 'util'] = perf_specs['util']
        df.loc[idx, 'ops_cycle'] = perf_specs['ops_cycle']
        df.loc[idx, 'insts_cycle'] = perf_specs['insts_cycle']
        
        df.loc[idx, 'max_mregs'] = perf_specs['max_mregs']
        df.loc[idx, 'max_mrf_capacity'] = perf_specs['max_mrf_capacity']
        # df.loc[idx, 'l3_size'] = perf_specs['l3_size']
        df.loc[idx, 'mem_bw'] = perf_specs['mem_bw']
        df.loc[idx, 'max_mem_bw'] = perf_specs['max_mem_bw']
        df.loc[idx, 'mrf_bw'] = perf_specs['mrf_bw']
        df.loc[idx, 'mrf_capacity'] = perf_specs['mrf_capacity']
        df.loc[idx, 'mrf_gates'] = perf_specs['mrf_gates']
        df.loc[idx, 'macc_gates'] = perf_specs['macc_gates']
        df.loc[idx, 'opu_gates'] = perf_specs['opu_gates']
    return df

# Use `init_pm` to initialize model with desired input ranges. Defaults are scalars to allow for easy sweeping of one variable.
def init_pm(
    databits = np.array([64]),
    t_mem = np.array([20]),     # [cycles]
    M = np.array([16]),         # [num rows]
    N = np.array([16]),         # [num rows]
    K = np.array([16]),         # [num rows]
    l2_cache = np.array([256]), # [KBytes]
    kl = np.array([1]),         # [num rows]
    vlB = np.array([256])/8,    # [Bytes]
    mlB = np.array([256])/8,    # [Bytes]
    num_mregs = np.array([2]),
    t_op = np.array([0]),     # [cycles]
    width_mmu = np.array([1]),     # [1 or 1/2]
    ):
    return generate_df(databits, t_mem, M,N,K, l2_cache, kl, vlB, mlB, num_mregs, t_op, width_mmu)
