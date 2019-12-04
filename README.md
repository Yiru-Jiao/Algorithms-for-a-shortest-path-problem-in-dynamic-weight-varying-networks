# Code for "A novel model and its algorithms for the shortest path problem of dynamic weight-varying networks in Intelligent Transportation Systems"
To repeat the expirement, execute code below in Matlab (at first ensure all .m files and .mat files are in the same folder):

The improved adaptive ant colony algorithm: VWA(D,V,C,delta_t0,1,6,10,5,2,0.5,0.5,0.8)

The brute-force algorithm based on matrix expansion: VWN(D,V,C,delta_t0,1,6)

P.S. delta_0 was gotten from a public dataset named Oliver30 for the shortest path problem.

--
I'm so sorry I could not remember and provide more details for the project ran nearly 3 years ago...

# Citation
Zhongzhong Jiang, Yiru Jiao, Ying Sheng, Xiaohong Chen. (2017). A novel model and its algorithms for the shortest path problem of dynamic weight-varying networks in Intelligent Transportation Systems. Journal of Intelligent & Fuzzy Systems, 33(5), 3095-3102.

# Abstract
Intelligent Transportation Systems (ITS) are deﬁned as those systems utilizing synergistic technologies and systems engineering concepts to develop and improve transportation systems. In this paper, a novel route selection problem based on the envisaged driving mode with dynamic signals in ITS is proposed. It belongs to a kind of the shortest path problem of dynamic weight-varying networks, and the arc-weights of the network vary with the arc-chosen, so it cannot be solved by the existing greedy algorithms. According to the characteristics of the proposed problem, ﬁrstly, a dynamic programming model for the driving mode on a single path is established. Secondly, two algorithms for solving the route selection problem based on the former mode are developed. One is a brute-force algorithm based on matrix expansion with the computational complexity of O(Nt ×n^2). The other is an improved adaptive ant colony algorithm with the complexity of O(Nc×m×n^2). Finally, the computational experiments show the advantages and disadvantages of the two effective algorithms with numerical examples.
