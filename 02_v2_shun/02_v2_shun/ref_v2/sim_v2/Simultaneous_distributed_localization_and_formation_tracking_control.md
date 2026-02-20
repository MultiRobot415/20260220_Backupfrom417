Automatica175(2025)112188
Contents lists available at ScienceDirect
Automatica
journal homepage: www.elsevier.com/locate/automatica
Simultaneousdistributedlocalizationandformationtrackingcontrol
viamatrix-weightedpositionconstraints✩
XuFanga,b,c,LihuaXieb,∗,DimosV.Dimarogonasc
aKey Laboratory of Intelligent Control and Optimization for Industrial Equipment of Ministry of Education, Dalian University of
Technology, Dalian 116024, China
bSchool of Electrical and Electronic Engineering, Nanyang Technological University, 639798, Singapore
cSchool of Electrical Engineering and Computer Science, KTH Royal Institute of Technology, Stockholm SE 10044, Sweden
a r t i c l e i n f o
Article history:
Received22June2023
Receivedinrevisedform28September2024
Accepted6January2025
Availableonline12February2025
Keywords:
Formationtrackingcontrol
Distributedlocalization
Bearing
Ratio-of-distance
Multi-agentsystem
3-Dspacea b s t r a c t
This paper studies the problem of 3-D relative-measurement-based leader–follower simultaneous
distributedlocalizationandformationtrackingcontrol.Thepositioninformationisonlyavailableto
theleaders,andthefollowershaveinter-agentrelativemeasurementsandcommunicationwiththeir
neighbors. The key contribution is the development of a weight-matrix-based position constraint,
which can make use of relative measurements such as bearing, ratio-of-distance, angle, distance,
relative position and their mixture to describe the position relationship among each follower and
its neighbors in 3-D space. A bearing-based distributed protocol is proposed for each follower to
estimateitspositionandtrackitstargetposition,whichcandrivethefollowersfromtheirunlocalizable
positions to localizable positions. The proposed algorithm is then extended to the case that both
bearingandratio-of-distancemeasurementsareavailable,wherethefollowersarelocalizableatall
timesifthefollowersandtheirneighborsarenotcollocated.Inaddition,theproposedmethodis
alsoapplicabletohomogeneousorheterogeneousangle,distance,andrelativepositionmeasurements
astheratio-of-distancesorbearingscanbeobtainedindirectlybytheserelativemeasurements.A
remarkableadvantageisthattheproposedmethodcanbeimplementedwithoutpersistentlyexciting
motions.Someillustrativesimulationsarepresentedtoverifythetheoreticalresults.
©2025ElsevierLtd.Allrightsarereserved,includingthosefortextanddatamining,AItraining,and
similartechnologies.
## 1. Introduction
Network localization and formation tracking control for
multi-agent systems enjoy wide application in civilian and mili-
tary applications such as the well-studied target entrapping (Yang
et al.,2021,2020). In leader–follower multi-agent systems, net-
work localization aims to localize the followers based on the
leaders’ positions and relative measurements. The widely used
inter-agent relative measurements in distributed network local-
ization are: distances (Aspnes et al.,2006;Eren et al.,2004;
Khan et al.,2009), angles (Jing et al.,2022), bearings (Le-Phan
et al.,2023;Shames et al.,2012;Zhao & Zelazo,2016), and their
mixture (Barooah & Hespanha,2007). Different from network
✩This work was partially supported by the Wallenberg AI, Autonomous
Systems and Software Program (WASP) funded by the Knut and Alice Wallenberg
Foundation and Ministry of Education, Singapore, under AcRF TIER 1 Grant
RG64/23. The material in this paper was not presented at any conference. This
paper was recommended for publication in revised form by Associate Editor
Charalampos Bechlioulis under the direction of Editor Christos G. Cassandras.∗Corresponding author.
E-mail addresses:fa0001xu@e.ntu.edu.sg (X. Fang),elhxie@ntu.edu.sg
(L. Xie),dimos@kth.se(D.V. Dimarogonas).localization, formation tracking control aims to track target mov-
ing formations based on inter-agent relative measurements and
communication.
In this article, we focus on utilizing homogeneous or
heterogeneous relative measurements (such as bearing or ratio-
of-distance measurement) to achieve simultaneous distributed
localization and formation tracking control. Most of the
existing bearing-based research only studies formation tracking
control (Trinh & Ahn,2021). Note that the global (absolute) posi-
tions of the agents are also important in certain application tasks
such as map construction and source seeking (Chen et al.,2022).
Hence, there is a need of a simultaneous network localization and
formation tracking control scheme.
However, the bearing-based simultaneous network localiza-
tion and formation tracking control problem cannot be solved
trivially by combining the existing bearing-based network
localization (Le-Phan et al.,2023;Shames et al.,2012;Zhao &
Zelazo,2016) and formation tracking control approaches (Trinh
et al.,2018;Van Tran, Trinh et al.,2018;Zhao et al.,2019). The
reason is that the existing bearing-based network localization
methods require the network to be non-degenerate, e.g., each
follower and its neighbors are non-collinear or the network is
infinitesimally bearing rigid at all times (Le-Phan et al.,2024;
https://doi.org/10.1016/j.automatica.2025.112188
0005-1098/ ©2025 Elsevier Ltd. All rights are reserved, including those for text and data mining, AI training, and similar technologies.
X. Fang, L. Xie and D.V. Dimarogonas Automatica 175 (2025) 112188
Van Tran, Park, & Ahn,2018;Zhao & Zelazo,2019), which is
challenging to ensure. Thus, the first technical challenge is how
to realize bearing-based distributed localization in multi-agent
systems with dynamic (moving) agents.
To tackle this problem, the existing works (Su et al.,2023;
Tang & Loría,2022;Yang et al.,2021;Ye et al.,2017;Zhu et al.,
2023) usually require the agents to keep persistently exciting
motions. The persistently exciting motions inSu et al.(2023),
Tang and Loría(2022),Yang et al.(2021),Ye et al.(2017),Zhu
et al.(2023) are mainly used for the agents to achieve self-
localization, which restrict the design of desired target formation.
Also, they may increase the agents’ energy consumption. For ex-
ample, the agents need not only to perform translational motions
for target tracking, but also require additional circular motions
for localization (Ye et al.,2017). Thus, the second technical chal-
lenge is how to realize bearing-based distributed localization
and control in a motion-excitation-free manner. Although our
complex-Laplacian-based approach does not require persistently
exciting motions (Fang et al.,2023), it can only be applied in 2-D
plane and needs additional distance measurements. In addition,
the existing orthogonal-matrix-based position constraint (Zhao &
Zelazo,2016) is only applicable to bearing measurements. The
third technical challenge is to propose a general position con-
straint, which can accommodate different relative measurements
for distributed localization and formation control.
To deal with the above challenges, in this article, we propose
a 3-D simultaneous network localization and formation track-
ing control algorithm. The main contributions and novelty are
summarized as follows:
(i)Different from the bearing-based orthogonal projection
matrix (Zhao & Zelazo,2016), a relative-measurement-
based weight matrix is proposed to describe the position
relationship among each follower and its neighbors in 3-D
space. The proposed weight matrix is not only applicable
to bearing measurements, but also applicable to ratio-
of-distance, relative position, distance, angle, and their
mixture.
(ii)Based on the weight-matrix-based position constraint, a
3-D bearing-based simultaneous distributed localization
and formation tracking control protocol is proposed for
the followers to achieve self-localization and track their
target positions. The proposed algorithm is then extended
to the case where both bearing and ratio-of-distance mea-
surements are available. Different from motion-excitation-
based methods (Su et al.,2023;Tang & Loría,2022;Yang
et al.,2021;Ye et al.,2017;Zhu et al.,2023), the proposed
approach can be implemented in a motion-excitation-free
manner, i.e., the agents are not required to keep persis-
tently exciting motions.
(iii)Different from the existing bearing-based localization ap-
proaches which require the agents to be localizable at all
times (Le-Phan et al.,2023;Shames et al.,2012;Zhao & Ze-
lazo,2016), the proposed bearing-based protocol can drive
the followers out of unlocalizable positions and get them
to their target positions. In the bearing-ratio-of-distance-
based distributed protocol, each follower is localizable at
all times if each follower and its neighbors are not collo-
cated, i.e., each follower is localizable even if it is collinear
with its neighbors.
(iv)The sum of the squares of the estimation and tracking
errors of the followers will not increase when there are
occlusions in the environment or the followers are in
unlocalizable regions. In addition, the bearing-based or
bearing-ratio-of-distance-based distributed protocol can be
extended to homogeneous or heterogeneous angle, dis-
tance, and relative position measurements.The organization of this article is as follows. Section2intro-
duces the notations and formulates the problem. The concept
of weight-matrix-based position constraint is introduced in Sec-
tion3. Two distributed protocols are presented in Section4to
realize self-localization and track the desired position simulta-
neously. Some simulations are given in Section5to verify the
theoretical results. Conclusions are drawn in Section6.
## 2. Notations and problem statement
### 2.1. Notations
The set of real numbers and positive real numbers are denoted
byRandR+, respectively. The identity matrix is denoted by
Id∈Rd×d. Let| · |be the absolute value of a given real number.
Denote by1d,0d∈Rdthe column vectors with all entries equal to
one and zero, respectively. Let∥ · ∥1and∥ · ∥2be theL1-norm and
L2-norm of a given matrix or vector, respectively. Let det(·) be the
determinant of a given square matrix. A communication graph G
is a pair of{V,E}, where Vis a nonempty finite set of agents and
E⊆V×Vis a set of edges. Agent jis called a neighbor of agent
iif (i,j)∈E. Denote byATthe transpose of a matrix A∈Rd×dor
a vector A∈Rd. Denote byA∈ [R]k1×k2
d×da block matrix with k1
row block partitions and k2column block partitions, where each
block is a submatrix of dimension d×d. That is, the block matrix
A∈ [R]k1×k2
d×dis a matrix of dimension k1d×k2d. For example,
A=[
A11 A12
A21 A22]
∈ [R]2×2
3×3=R6×6, (1)
where the block matrix Ain(1)has 2 row block partitions and
2 column block partitions, and A11,A12,A21,A22∈R3×3are the
matrices of dimension 3×3.
Consider a group ofnmobile agents inR3, where pi=
[xi,yi,zi]T∈R3represents the position of agent iin 3-D space
and p= [pT
1, . . . ,pT
n]T∈R3nis the configuration of all agents. A
formation is denoted by (G,p). Suppose there aremleaders and
n−mfollowers. The set of leader group and follower group are
denoted byVl= {1, . . . ,m}andVf= {m+1, . . . ,n}, respectively.
The positions of leader group and follower group are denoted by
pl= [pT
1, . . . ,pT
m]Tand pf= [pT
m+1, . . . ,pT
n]T, respectively. The 3-D
inter-agent bearing gijin the global coordinate frame and ratio-
of-distance rijkfor the non-collocated agents i,j,kare represented
by
gij=pij
∥pij∥2,rijk=∥pij∥2
∥pik∥2, (2)
where pij=pj−piis the relative position between agent iand agent
j. Letˆpibe the position estimate of agent i. The angle θifor the
non-collocated agents i,j,kis represented byθi=arccos( gT
ijgik).
### 2.2. Graph design
The directed graph Gin this article is designed based on the
following rules:
(i)The agent setVofGconsists ofκ >1 subsets V1,V2,
· · ·,Vκ, where Vi∩Vj= ∅ifi̸ =j. Agent iis said to be in
layer sifi∈Vs(1≤s≤κ). Subset V1includes all leaders,
i.e.,V1=Vl. The union of the subsets V2, . . . ,Vκincludes
all followers, i.e.,⋃κ
s=2Vs=Vf;
(ii)There are at least two leaders, and the leaders have no
neighbor. Each follower ihas at least two neighbors, and
the neighbor setNiof follower iis designed as
Ni= {j∈s⋃
g=1Vg:j<i,(i,j)∈E,i∈Vs,s>1}.(3)
An example of the directed graph Gis given inFig. 1.
2
X. Fang, L. Xie and D.V. Dimarogonas Automatica 175 (2025) 112188
Fig. 1.A directed graph.
Fig. 2.Example of constructing a weight-matrix-based position constraint.
### 2.3. Problem statement
The dynamics of each agent is
̇pi=ui,i=1, . . . ,n, (4)
where pi∈R3is the position and ui∈R3is the control input.
In this article, we aim to localize and control the followers pf
based on the positions of the leaders pland inter-agent relative
measurements, i.e.,{lim
t→∞(ˆpf(t)−pf(t))=0,
lim
t→∞(pf(t)−p∗
f(t))=0,(5)
where ˆpfand p∗
fare, respectively, the position estimate and
target position of the follower group. The target trajectory p∗
f(t)
is assumed to be first-order differentiable.
## 3. Bearing-induced position relationship
### 3.1. Weight-matrix-based position constraint
If agents i,j,kare not collocated, i.e.,pi̸ =pj̸ =pk, a weight-
matrix-based position constraint for agents i,j,kis given by
Hij(pj−pi)+Hik(pk−pi)=0,i∈Vf, (6)
where Hij,Hik∈R3×3are appropriate weight matrices. The weight
matrices Hij,Hikin(6)can be calculated based on the positions
pi,pj,pkas shown in Algorithm2. A simple example of(6)is
given inFig. 2, where the positions of agents i,j,kare pi=
[0,0,0]T,pj= [2,1,1]Tand pk= [1,2,2]T. Then, the weight
matrices Hij,Hikin(6)are calculated as
Hij=[0.5 1.5 0
−1.5 2.5 0
0 0 2]
,Hik=[0.5−1.5 0
1.5−0.5 0
0 0 −1]
.(7)Eq.(6)can be rewritten as
Hiipi=Hijpj+Hikpk, (8)
where Hii=Hij+Hik. IfHiiis nonsingular, we have pi=H−1
ii(Hijpj+
Hikpk). Hence, the weight matrices Hij,Hikdescribe the position
relationship among agents i,j,k. The immediate question is when
the matrix Hiiis nonsingular.
Remark 1.As shown in Algorithm2, the weight matrices Hij,Hik
in(6)are calculated based on the plain positions of agents i,j,k,
which are different from the bearing-based orthogonal projection
matrix inTrinh et al.(2018),Zhao and Zelazo(2016).
Lemma 3.1. If agents i,j,k are not collocated and the weight
matrices Hij,Hikin(6)are calculated based on Algorithm 2, the
weight matrix Hii=Hij+Hikis nonsingular, i.e., pican be uniquely
determined by the positions pj,pkin(8).
The proof ofLemma 3.1is given inAppendix B.
Remark 2.InFig. 2, the agents i,j,kare not collocated, and we
can know from(7)that Hii=Hij+Hikis nonsingular. Hence, pican
be uniquely determined bypj,pk, i.e.,pi=Hijpj+Hikpk.
Definition 1.The configurations ˇp= [pT
i,pT
j,pT
k]Tandˇq=
[qT
i,qT
j,qT
k]Tare translation-scaling similar if they satisfy
ˇq=κsˇp+13⊗b, (9)
where κs∈R+is a non-zero scaling parameter, b∈R3is a
translation parameter, and⊗is the Kronecker product.
Lemma 3.2. The configurations ˇq= [qT
i,qT
j,qT
k]Tandˇp=
[pT
i,pT
j,pT
k]Thave the same weight matrices if they are translation-
scaling similar.
Proof.From(9), it has
qi=κspi+b,qj=κspj+b,qk=κspk+b. (10)
Combining(8)and(10), it has
Hiiqi−Hijqj−Hikqk
=Hii(κspi+b)−Hij(κspj+b)−Hik(κspk+b)
=κs(Hiipi−Hijpj−Hikpk)+(Hii−Hij−Hik)b
=κs(Hiipi−Hijpj−Hikpk)
=0.(11)
Since κs̸ =0, from(11), it has
Hiipi−Hijpj−Hikpk=0⇐ ⇒ Hiiqi−Hijqj−Hikqk=0.(12)
It is clear from(12)that the configurations ˇq= [qT
i,qT
j,qT
k]T
andˇp= [pT
i,pT
j,pT
k]Tcan have the same weight matrices. □
Definition 2.A formation (G,p) is said to be localizable if the
followers’ positions pfcan be uniquely determined by the leaders’
positions plthrough the weight-matrix-based position constraints
(6).
FromLemma 3.2, we can deduce that the proposed weight-
matrix-based position constraint between each follower and its
neighbors can be obtained if the translation-scaling similar con-
figuration can be calculated based on their relative measurements
as shown in subsequent sections.
3
X. Fang, L. Xie and D.V. Dimarogonas Automatica 175 (2025) 112188
Fig. 3.Agents i,j,kare not collocated but collinear.
### 3.2. Construction of weight-matrix-based position constraint using
bearing measurements
There are two cases for the non-collocated agents i,j,k:
(i)If agents i,j,kare not collocated but collinear, the bearings
among the agents i,j,kare not only invariant to trans-
lation and scaling ofpi,pj,pk, but also invariant to some
other motions ofpi,pj,pk. For example, the agents i,j,k
inFig. 3(a) andFig. 3(b) have the same bearings, but the
agents i,j,kinFig. 3(a) cannot be obtained by the trans-
lation and scaling of agents i,j,kinFig. 3(b). Thus, the
geometric shape of agents i,j,kcannot be uniquely de-
termined by the bearings if they are collinear, i.e., agent i
cannot be localized by agents j,k. Then, the weight matri-
cesHij,Hikin(6)are set toHij=0,Hik=0, which mean
that the position relationship among agents i,j,kis not
available.
(ii)If agents i,j,kare non-collinear, the bearings among the
agents i,j,kare only invariant to translation and scaling of
pi,pj,pk. Thus, the translation-scaling similar configuration
of agents i,j,kcan be obtained based on the bearings as
shown in Algorithm3. Then, the weight matrices Hij,Hikin
(6)can be calculated based on the translation-scaling sim-
ilar configuration through Algorithm2. FromLemma 3.1,
agent ican be localized by agents j,k.
The weight-matrix-based position constraint of each follower
i∈Vffrom(8)can be aggregated into a block matrix form, i.e.,
Bfp=0, (13)
where p= [pT
1, . . . ,pT
n]TandBf∈ [R](n−m)×n
3×3in(13)is a block
matrix with n−mrow block partitions and ncolumn block
partitions. Let[Bf]ijbe a block in theith row block partition i∈Vf
and jth column block partition, i.e.,
[Bf]ij=⎧
⎨
⎩Hii, j=i,
0, j/∈Ni,j̸ =i,
−Hij,j∈Ni,j̸ =i,(14)
where Hii=Hij+Hik,j,k∈Ni, i.e., follower ichooses two
neighbors for constructing Hii. Based onpland pf,Bf= [BflBff]in
(13)can be decoupled into
Bflpl+Bffpf=0, (15)
where Bfl∈ [R](n−m)×m
3×3andBff∈ [R](n−m)×(n−m)
3×3. If the block matrix
Bffin(15)is nonsingular, it has pf= −B−1
ffBflpl. The construc-
tion of weight-matrix-based position constraint using bearing
measurements is shown in Algorithm1. For the bearing-based
formation,Definition 2becomes
Definition 3.A bearing-based leader–follower formation (G,p)
is said to be localizable if the block matrix Bffin(15)is nonsin-
gular, i.e., the positions of the follower group pfcan be uniquelydetermined by those of the leader group plbased on inter-agent
bearings.
Algorithm 1Construction of Weight-matrix-based Position
Constraint Using Bearing Measurements
1:Available information: bearing measurements among each
follower iand its neighbors j,k∈Ni;
2:Forfollower i=m+1:n
3: Ifagents i,j,k(j,k∈Ni) are non-collinear do
4:Based on bearing measurements, calculate Hij,
5: Hikthrough Algorithm2and Algorithm3;
6: Else, do
7:The weight matrices are set toHij=0,Hik=0;
8: End
9:End
10:Aggregating the position constraint of each follower iinto a
block matrix form(13);
11:Obtaining the position relationship(15)between pland pf.
## 4. Bearing-based distributed localization and formation track-
ing control
To achieve distributed localization, the localizable condition of
bearing-based multi-agent systems needs to be explored. That is,
how to guarantee the block matrix Bffin(15)to be nonsingular.
From Section2.2and(3), the block matrix Bffin(15)with our
designed directed graph topology Gis
Bff=⎡
⎢⎢⎣H(m+1)(m+1) 0
−H(m+2)(m+1) H(m+2)(m+2)
.........
−Hn(m+1) −Hn(m+2) · · · Hnn⎤
⎥⎥⎦.(16)
### 4.1. Localizability of bearing-based multi-agent system
Theorem 4.1. A bearing-based formation(G,p)with a directed
graph topology is localizable if each follower i∈Vfand its neighbors
j,k∈Niare not collinear.
Proof.As shown in (ii) of Section3.2, if each follower i∈Vfand its
neighbors j,k∈Niare non-collinear, the weight matrices Hij,Hik
of agents i,j,kin(6)can be calculated based on inter-agent bear-
ings through Algorithm2and Algorithm3. Since agents i,j,kare
non-collinear, agents i,j,kare not collocated. FromLemma 3.1
and(16), for the diagonal submatrices Hii,i=m+1, . . . ,n, it has
det(Hii)̸ =0,i=m+1, . . . ,n. (17)
Thus, it has
det(Bff)=n∏
i=m+1det(Hii)̸ =0. (18)
Hence, the block matrix Bffin(15)is nonsingular. That is, the
bearing-based formation (G,p) is localizable. □
In this article, the bearing-based formation (G,p) is not as-
sumed to be localizable at all times shown in the following
section.
### 4.2. Bearing-based distributed protocol
Definition 4.The graph ¯G=(V,¯E) is an induced graph of
G=(V,E) with induced edge set¯E=E∪˜E, where ˜E= {(j,i):
(i,j)∈E,i,j∈Vf}is the augmented edge set.
4
X. Fang, L. Xie and D.V. Dimarogonas Automatica 175 (2025) 112188
Fig. 4.An induced graph ofGinFig. 1.
The induced graph ¯G(such asFig. 4) is obtained by making the
directed edges among the followers in the graph G(such asFig. 1)
undirected.
Assumption 1.The target position p∗
i(t) of each agent iis first-
order differentiable. The relative measurements are available.
Each leader i∈Vlcan access its position and is located at its target
position. Each follower i∈Vfcan access the target position p∗
i(t)
and its derivative ̇p∗
i(t), where the target bearings g∗
ij,g∗
ik,j,k∈Ni
are not parallel. The initial positions of the followers are within a
known bounded area.
Remark 3.The case that each follower i∈Vfhas no access to
the target position p∗
i(t) and its derivative ̇p∗
i(t) can be solved
by employing a parameter passing technique (Chen et al.,2022).
Compared to our recent hybrid control design inFang et al.
(2024), the proposed method has no dwell time or Zeno behavior
problem and can have fewer neighbors and communication.
FromTheorem 4.1, a bearing-based multi-agent system (G,p)
is localizable if each follower i∈Vfand its neighbors j,k∈Niare
non-collinear. Considering that it is impractical to assume that
each follower i∈Vfand its neighbors j,k∈Niare non-collinear
at all times. Inspired byTrinh et al.(2018) and our previous
work (Chen et al.,2022), we present a distributed protocol with
an induced graph ¯G=(V,¯E) to drive each follower i∈Vfout of
the undesired collinear (unlocalizable) positions and get it to its
target position, i.e.,⎧
⎪⎪⎪⎨
⎪⎪⎪⎩ui= −w(ˆpi−p∗
i)+̇p∗
i−τi(sign(ˆpi−p∗
i)−λi),
̇ˆpi= −2w(ˆpi−p∗
i)+̇p∗
i+ξijk+∑
i,g∈Ns,s∈Vfξsig
−2τi(sign(ˆpi−p∗
i)−λi),(19)
where w∈R+and sign(·) is the signum function defined
component-wise. τi= ∥gij−g∗
ij∥2+ ∥gik−g∗
ik∥2≥0.λi∈R3is
a non-zero adjustment term with 0<∥λi∥2<1. The neighbor
set of each follower is defined in(3)with a directed graph G, and
ξijk=HT
iiHij(ˆpj−ˆpi)+HT
iiHik(ˆpk−ˆpi),
ξsig=HT
siHss(ˆps−ˆpi)−HT
siHsg(ˆpg−ˆpi).(20)
Remark 4.The variable τican also be designed asτi=(1−
sign(∥Hii∥1))(∥gij−g∗
ij∥2+ ∥gik−g∗
ik∥2)≥0. We can deduce that
τi=0 if follower iis localizable.
Remark 5.The weight matrices Hii,Hij,Hik,Hss,Hsi,Hsgcan be
obtained by bearing measurements as shown in Section3.2. Al-
though the neighbor set of each follower iis defined in(3)witha directed graph G, the proposed distributed protocol (19) is
implemented in an induced graph ¯Gas shown inDefinition 4.
For each follower i∈Vf, the information ξijkis obtained based on
bearings along with communication with its neighbors j,k∈Ni
through the edges inE, while the information ξsigis obtained
based on communication with follower sthrough the augmented
edges in˜E, where i,g∈Ns.
Remark 6.The idea is that the position estimator provides posi-
tion information for the controller, while the controller helps en-
sure that the system is localizable. The adjustment termλiis any
bounded three-dimensional vector satisfying 0<∥λi∥2<1. For
example, λican be designed asλi=1
2[sin(τi),cos(τi),tanh(τi)]T,
where tanh(·) is the hyperbolic tangent function.
Let¯ef=pf−p∗
f∈R3(n−m)be the tracking errors of the follower
group. Letˆef=ˆpf−pf∈R3(n−m)be the position estimation errors
of the follower group. Denote byψi= −τi(sign(ˆpi−p∗
i)−λi). Note
that(19)can then be rewritten in a compact form as{̇pf= −w(ˆpf−p∗
f)+̇p∗
f+ψf,
̇ˆpf= −2w(ˆpf−p∗
f)+̇p∗
f−Dffˆpf−Dflpl+2ψf,(21)
where ψf= [ψT
m+1, . . . , ψT
n]Tand
Dff=BT
ffBff,Dfl=BT
ffBfl. (22)
The block matrices Bff,Bflin(22)are from(15). We get from
(15)that Dffˆpf+Dflpl=Dffˆef. Then,(22)becomes[̇¯ef
̇ˆef]
= −[wI3(n−m)wI3(n−m)
wI3(n−m)wI3(n−m)+Dff][¯ef
ˆef]
+[ψf
ψf]
.(23)
UnderAssumption 1, the initial positions of the followers are
within a known bounded area, i.e., the upper bound
∥[¯eT
f(0),ˆeT
f(0)]∥2is known for the implementation of the dis-
tributed protocol. Letϱfbe the upper bound of∥[¯eT
f(0),ˆeT
f(0)]∥2
as
ϱf≥ ∥[¯eT
f(0),ˆeT
f(0)]∥2. (24)
Define the parameter φi∈Rof each agent ias
φi={
0,i∈Vl,
ϱf,i∈Vf.(25)
Theorem 4.2. Under Assumption 1, the bearing-based simultane-
ous distributed localization and formation tracking control protocol
(19)with an induced graph ¯Gcan avoid inter-agent collision and
achieve the localization and control objective(5)if for any i,j∈V,
the following two conditions hold:
(i)∥pi(0)−pj(0)∥2̸ =0;
(ii)∥p∗
i(t)−p∗
j(t)∥2> φi+φj, where φi, φjare given in(25).
Remark 7.The two conditions (i)-(ii) are used for inter-agent col-
lision avoidance. The condition (ii) can be satisfied by tuning the
target positions p∗
i(t) and p∗
j(t). Based on the conditions (i)-(ii),
the proposed distributed protocol in(19)will achieve semi-global
stability (Teel,1995). In addition, if the initial positions of all
followers are available, the initial position estimates and initial
target positions of the followers can be set toˆpf(0)=pf(0)=p∗
f(0),
and thenϱfin(24)can be set to zero.
Proof.For the discontinuous controller(19), the Filippov solu-
tions of(19)exist as the signum function sign(·) is measurable
and locally essentially bounded (Filippov,2013). For the Lya-
punov function candidate V1=1
2(¯eT
f¯ef+ˆeT
fˆef),̇V1exists almost
5
X. Fang, L. Xie and D.V. Dimarogonas Automatica 175 (2025) 112188
everywhere, i.e.,
̇V1∈a.e.̇˜V1, (26)
where a.e. represents ‘‘almost everywhere’’ anḋ˜V1is the set-
valued Lie derivative ofV1(Fischer et al.,2013), i.e.,
̇˜V1= −[¯eT
fˆeT
f][wI3(n−m)wI3(n−m)
wI3(n−m)wI3(n−m)+Dff][¯ef
ˆef]
−n∑
i=m+1τi(ˆpi−p∗
i)T(K[sign(ˆpi−p∗
i)] −λi)
= −w∥¯ef+ˆef∥2
2−ˆeT
fDffˆef
−n∑
i=m+1τi(∥ˆpi−p∗
i∥1−(ˆpi−p∗
i)Tλi),(27)
where K[·]is a Filippov set-valued mapping function. We have
used the fact that x·K[sign( x)] = {|x|},x∈Rto obtain the last
equality of(27). Thus,̇˜V1is singleton anḋV1=̇˜V1. Then,(27)
becomes
̇˜V1≤ −w∥¯ef+ˆef∥2
2−ˆeT
fDffˆef−n∑
i=m+1τi(1− ∥λi∥2)∥ˆpi−p∗
i∥1
≤0.(28)
We get from(28)that
∥[¯eT
f(t),ˆeT
f(t)]∥2≤ ∥[¯eT
f(0),ˆeT
f(0)]∥2,t≥0. (29)
UnderAssumption 1, we get from(25)and(29)that
∥pi(t)−p∗
i(t)∥2≤φi. (30)
For any i,j∈Vand t≥0, it has
∥pi(t)−pj(t)∥2
≥ ∥p∗
i(t)−p∗
j(t)∥2− ∥pi(t)−p∗
i(t)∥2− ∥pj(t)−p∗
j(t)∥2
≥ ∥p∗
i(t)−p∗
j(t)∥2−φi−φj>0.(31)
Hence, there will be no inter-agent collision iḟ˜V1≤0. Note
thaṫ˜V1=0 if and only if
Dffˆef=0,ˆpf=p∗
f. (32)
Eq.(32)is equivalent to
Dffˆef=0,ˆef+¯ef=0. (33)
Based on(23)and(33), it has
̇ˆef+̇¯ef= −[τm+1λT
m+1, τm+2λT
m+2, . . . , τnλT
n]T. (34)
Ifˆef̸ =0, the block matrix Dffin(32)is singular and there is at
least one follower i∈Vfthat is unlocalizable, i.e., the term τi>0
in(19)is positive. Since the time-varying adjustment term λiin
(19)is designed to be nonzero, it hasτiλi̸ =0. Then, it is clear
from(34)thaṫˆef+̇¯ef̸ =0, which implies thatˆef+¯ef=0in(33)
cannot hold for all times. Hence, the configuration corresponding
toˆef̸ =0,ˆef+¯ef=0is not an equilibrium of(23)due to the
nonzero adjustment term λi. From the nonsmooth corollaries of
the LaSalle–Yoshizawa Theorem (Fischer et al.,2013), the fol-
lower group converges toˆef=0,ˆef+¯ef=0, i.e., the localization
and control objective ˆef=0,¯ef=0(5)is achieved. □
Remark 8.If the adjustment term λiis designed to be zero,
we can know from(34)that the configuration corresponding to
ˆef̸ =0,ˆef+¯ef=0is also an equilibrium of(23). For this case, if
the followers are unlocalizable, they will keep unlocalizable. Notethat the unlocalizable positions of the followers are not the equi-
librium of(23)ifλi̸ =0. From the nonsmooth corollaries of the
LaSalle–Yoshizawa Theorem (Fischer et al.,2013), the followers
will converge to the equilibrium of(19), i.e., the followers will
not go back to the unlocalizable positions.
### 4.3. Bearing-ratio-of-distance-based distributed protocol
To design a continuous controller, the strategy is to combine
bearing and ratio-of-distance measurements that are available
by low-cost onboard vision (camera) sensors (Mehdifar et al.,
2022). In bearing-based localization, agent icannot be localized
by agents j,kif agents i,j,kare not collocated but collinear.
Compared with bearing-based localization, one remarkable ad-
vantage of bearing-ratio-of-distance-based localization is that the
translation-scaling similar configuration of agents i,j,kcan be
obtained even if agents i,j,kare not collocated but collinear
as shown inAppendix D. That is, agent ican be localized by
agents j,kat all times if agents i,j,kare not collocated. Then,
Assumption 1is relaxed as
Assumption 2.The target position p∗
i(t) of each agent iis first-
order differentiable. Each leader i∈Vlcan access its position and
located at its target position. Each follower i∈Vfcan access the
target position p∗
i(t) and its derivative ̇p∗
i(t). The initial positions
of the followers are within a known bounded area. The relative
measurements are available.
Based on bearing and ratio-of-distance measurements, the
distributed protocol of each follower i∈Vfin(19)is modified
as⎧
⎪⎨
⎪⎩ui= −w(ˆpi−p∗
i)+̇p∗
i,
̇ˆpi= −2w(ˆpi−p∗
i)+̇p∗
i+ξijk+∑
i,g∈Ns,s∈Vfξsig, (35)
where the terms ξijkandξsigare given in (20). The ratio-of-
distances and bearings are used to find translation-scaling similar
configuration shown inAppendix D. The weight matrices in(20)
are then calculated by the translation-scaling similar configura-
tion through Algorithm2.(35)can be rewritten in a compact form
as{̇pf= −w(¯ef+ˆef)+̇p∗
f,
̇ˆpf= −2w(¯ef+ˆef)+̇p∗
f−Dffˆpf−Dflpl,(36)
where
Dff=BT
ffBff,Dfl=BT
ffBfl. (37)
Theorem 4.3. Under Assumption 2, the bearing-ratio-of-distance-
based simultaneous distributed localization and formation tracking
control protocol(35)with an induced graph ¯Gcan avoid inter-agent
collision and achieve the localization and control objective(5)if
for any i,j∈V, the following two conditions hold:
(i)∥pi(0)−pj(0)∥2̸ =0;
(ii)∥p∗
i(t)−p∗
j(t)∥2> φi+φj, where φi, φjare given in(25).
Proof.Consider a Lyapunov function candidate V1=1
2(¯eT
f¯ef+
ˆeT
fˆef). We have
̇V1= −[¯eT
fˆeT
f][wI3(n−m)wI3(n−m)
wI3(n−m)wI3(n−m)+Dff][¯ef
ˆef]
≤0.(38)
Similar to the Proof ofTheorem 4.2, we can know that there
will be no inter-agent collision iḟV1≤0. We can know from
Appendix Dthat the translation-scaling similar configuration can
6
X. Fang, L. Xie and D.V. Dimarogonas Automatica 175 (2025) 112188
be calculated based on bearing and ratio-of-distance measure-
ments at all times. Then, fromLemma 3.2, the weight matrices
in(6)can be calculated based on the translation-scaling similar
configuration through Algorithm2at all times. FromLemma 3.1
and(16), we can deduce that the block matrix Bffis thus nonsin-
gular at all times, i.e., the block matrix Dff=BT
ffBff>0 is positive
definite for all times t≥0. Eq.(38)becomes
̇V1<0,if¯ef,ˆef̸ =0. (39)
Hence, the objective ˆef=0,¯ef=0(5)is achieved. □
### 4.4. Discussion on other types of relative measurements and appli-
cations in 2-D plane
The bearings or ratio-of-distances are used in the proposed
distributed protocol as shown in Sections4.2and4.3. Note that
the bearings or ratio-of-distances among each follower iand its
neighbors j,k∈Nican also be obtained indirectly by relative
position, distance, and angle measurements shown as follows: (i)
If the relative positions pij,pikare available, the bearings gij,gik
and ratio-of-distance rijkcan be obtained by(2); (ii) If the dis-
tances ∥pij∥2,∥pik∥are available, the ratio-of-distance rijkcan
be obtained by (2); (iii) If the angles θj, θkare available and
agents i,j,kare non-collinear, the ratio-of-distance rijkcan be
obtained byrijk=sinθk
sinθj. If agents i,j,kare collinear, i.e., the
ratio-of-distance rijkcannot be obtained by the angles θj, θk, the
weight matrices in (6) are set toHij=Hik=0. For this sit-
uation, the idea and technique in Section4.2can be utilized
to drive the agents out of the undesired collinear (unlocaliz-
able) positions. Hence, the proposed method is also applicable to
homogeneous or heterogeneous relative position, distance, and
angle measurements.
In addition, if agents i,j,kare not collocated in 2-D plane,
based on the method inLemma A.1, the 2-D weight matrices
Hij,Hik∈R2×2in(6)have the following form: Hij=[
c1c3
−c3c1]
and Hik=[
c2c4
−c4c2]
, where c1,c2,c3,c4∈Rin (41) are
calculated based on the 2-D positions pi,pj,pk. It can be verified
that the 2-D weight-matrix-based position constraint in (6) is
invariant to translation, rotation, and scaling ofpi,pj,pk. Hence,
the 2-D weight matrices Hij,Hikcan be calculated based on local
bearings or ratio-of-distances through Algorithms1–3. Since the
local bearings or ratio-of-distances can also be obtained indirectly
by local relative position, distance, and angle measurements, the
proposed weight-matrix-based position constraint is also applica-
ble to 2-D homogeneous or heterogeneous local relative position,
distance, and angle measurements.
### 4.5. Discussion on the occlusions in the environment
If the relative measurements (such as bearings or ratio-of-
distances) among agents i,j,kare unavailable due to the occlu-
sions in the environment, the weight matrices Hij,Hikin(6)are
set asHij=0,Hik=0and the term τiin(19)is set as any positive
real number. Then, Dffin(23)(or(38)) will be nonsingular and
̇˜V1≤0 (oṙV1≤0). Thus, the sum of the squares of the estimation
and tracking errors of the followers will not increase when there
are occlusions in the environment, i.e., the proposed controller is
robust to the occlusions in the environment. If the occlusions do
not occur at all times, the localization and control objective can
still be achieved under the proposed distributed protocol.
### 4.6. Comparison with existing works
The comparison with most related bearing-based distributed
localization and formation control (Su et al.,2023;Tang et al.,2020,2021;Tang & Loría,2022;Trinh et al.,2018;Van Tran, Trinh
et al.,2018;Yang et al.,2021;Zhao et al.,2019;Zhu et al.,2023)
is given below.
(i)The application scenarios are different. The worksSu et al.
(2023),Tang et al.(2020,2021),Tang and Loría(2022),
Trinh et al.(2018),Van Tran, Trinh et al.(2018),Yang et al.
(2021),Zhao et al.(2019),Zhu et al.(2023) are only applica-
ble to bearing measurements, whereas this article provides
a general distributed localization and control framework,
which can accommodate different relative measurements.
The bearing measurement is a special case in this article.
(ii)The requirements on the desired bearings or desired for-
mations are different. The worksSu et al.(2023),Tang
et al.(2020,2021),Tang and Loría(2022),Yang et al.
(2021),Zhu et al.(2023) require the desired bearings or
desired formations are persistently exciting. In this work,
the requirement on the persistently exciting motions can
be removed.
(iii)The requirements on the actual bearings are different. The
controllers inSu et al.(2023),Tang et al.(2020,2021),Tang
and Loría(2022),Trinh et al.(2018),Van Tran, Trinh et al.
(2018),Yang et al.(2021),Zhao et al.(2019),Zhu et al.
(2023) require that the actual bearings are measurable at
all times. In our work, the proposed method can still work
if the actual bearings are not measurable. For this case, the
sum of the squares of the estimation and tracking errors
of the followers will not increase. That is, the proposed
controller is robust to the occlusions in the environment.
(iv)The coordinate frames are different in 2-D plane. The bear-
ing measurements inSu et al.(2023),Tang et al.(2020,
2021),Tang and Loría(2022),Trinh et al.(2018),Van Tran,
Trinh et al.(2018),Yang et al.(2021),Zhao et al.(2019)
are measured in the global coordinate frame, whereas the
proposed weight-matrix-based position constraint can not
only be calculated by 2-D local bearing measurements, but
also by other types of 2-D homogeneous or heterogeneous
local relative measurements.
(v)The implementation conditions are different. The works
inTang et al.(2020,2021),Yang et al.(2021) require that
there is no inter-agent collision among the agents, whereas
the proposed controller can guarantee inter-agent collision
avoidance. The work inSu et al.(2023),Tang et al.(2021),
Van Tran, Trinh et al.(2018),Yang et al.(2021),Zhu et al.
(2023) needs additional relative velocities, relative orienta-
tions, velocity information of the leaders, or derivatives of
bearings, which is not required in this work. The worksSu
et al.(2023),Yang et al.(2021),Zhao and Zelazo(2016),
Zhu et al.(2023) are only applicable to 2-D plane or static
agents, whereas the proposed method is applicable to both
2-D and 3-D dynamic agents.
## 5. Simulation
In this section, we present two simulation examples in 3-D
space, where the formations consist of 2 leaders Vl= {1,2}and 3
followers Vf= {3,4,5}. The graph Gof the following case (i) and
case (ii) is given inFig. 1. The induced graph ¯Gfor implementing
the proposed distributed protocol(19)or(35)is given inFig. 4.
Case (i): If only the bearing measurements are available, the
followers can achieve the objective (5) under the proposed
bearing-based distributed protocol (19). The positions of the
followers 3,4,5 are collinear at some time instants as shown in
Fig. 5, where the adjustment termλiin bearing-based distributed
protocol(19)can drive the followers out of undesired collinear
(unlocalizable) condition and get to their target positions. The
7
X. Fang, L. Xie and D.V. Dimarogonas Automatica 175 (2025) 112188
Fig. 5.Trajectories of the agents in case (i).
Fig. 6.Position estimation errors in case (i).
position estimation errors and formation tracking errors converge
to zero as shown inFig. 6andFig. 7, respectively. The control
inputs are given inFig. 8.
Case (ii): If the bearing and ratio-of-distance measurements
are available, the followers can achieve self-localization and form
time-varying target formation under the bearing-ratio-of-
distance-based distributed protocol(35), where there is no inter-
agent collision among the agents as shown inFig. 9. We can
know fromFig. 10andFig. 11that both the position estima-
tion errors and formation tracking errors converge to zero. The
corresponding control inputs are given inFig. 12.
## 6. Conclusion
This paper addresses the problem of 3-D relative-measure-
ment-based simultaneous distributed localization and formation
tracking control. Two simultaneous distributed localization and
formation tracking control protocols are proposed for the fol-
lowers to achieve self-localization and track their target posi-
tions asymptotically, while the inter-agent collision is avoided
under some conditions. A remarkable advantage is that the pro-
posed method can be implemented without persistently exciting
motions.
The limitation of the proposed approach is that the global
coordinate frame is needed in 3-D space. The 3-D orientation
estimation algorithm may be used to remove the requirement
of global coordinate frame as inBoughellaba and Tayebi(2020).
Future work will focus on distributed localization and formation
tracking control with nonlinear agent dynamics and distance
measurements. FromAssumptions 1–2, we only require that the
Fig. 7.Tracking errors in case (i).
Fig. 8.Control inputs in case (i).
Fig. 9.Trajectories of the agents in case (ii).
target position p∗
i(t) of each agent ibe first-order differentiable.
There is no restriction on the transformation of target formation.
Thus, the target formation can be designed based on distance
rigidity. In addition, the uncertainty or disturbances could be
taken into consideration as inSu et al.(2023).
Appendix A. Calculation of the weight matrices Hij,Hik
Lemma A.1. Consider the 2-D vectors b1= [b11,b12]T, b2=
[b21,b22]T, b3= [b31,b32]T. If b1̸ =b2̸ =b3, the following inequality
8
X. Fang, L. Xie and D.V. Dimarogonas Automatica 175 (2025) 112188
Fig. 10.Position estimation errors in case (ii).
Fig. 11.Tracking errors in case (ii).
Fig. 12.Control inputs in case (ii).
holds:
(c1+c2)2+(c3+c4)2̸ =0, (40)
where
c1=b21−b11
∥b1−b2∥2
2,c2=b11−b31
∥b1−b3∥2
2,c3=b22−b12
∥b1−b2∥2
2,c4=b12−b32
∥b1−b3∥2
## 2. (41)Proof.For the 2-D vectors b1,b2,b3∈R2, we have[
c1c3
−c3c1]
(b2−b1)+[
c2c4
−c4c2]
(b3−b1)=0.(42)
If the inequality(40)does not hold, i.e.,c1= −c2and c3= −c4,
(42)becomes[
−c2−c4
c4−c2]
(b2−b1)+[
c2c4
−c4c2]
(b3−b1)=0.(43)
We get from(43)that[
c2c4
−c4c2]
(b3−b2)=0. From(41),
it has det([
c2c4
−c4c2]
)=1
∥b1−b3∥2
2̸ =0. Then, we can deduce
that b2=b3, which contradicts the fact that b2̸ =b3. Thus, the
inequality(40)holds. □
A.1. Plain position and plain-position-based parameters
The position of agent iin 3-D space ispi= [xi,yi,zi]T∈R3.
Define the plane positions of agent ias
αi= [xi,yi]T, βi= [yi,zi]T, γi= [zi,xi]T, (44)
where αi, βi, γi∈R2are, respectively, the plane positions of
agent iin X-Y plane, Y-Z plane, and Z-X plane. For agents j,k, their
plane positions areαj, βj, γjandαk, βk, γk. The plain-position-
based parameters s1, . . . ,s12in(45)will be used in the calcula-
tion of the weight matrices Hij,Hik.
s1=τ1(τ1+τ2)+τ3(τ3+τ4)
(τ1+τ2)2+(τ3+τ4)2,s2=−τ3(τ1+τ2)+τ1(τ3+τ4)
(τ1+τ2)2+(τ3+τ4)2,
s3=τ2(τ1+τ2)+τ4(τ3+τ4)
(τ1+τ2)2+(τ3+τ4)2,s4=−τ4(τ1+τ2)+τ2(τ3+τ4)
(τ1+τ2)2+(τ3+τ4)2,
s5=τ5(τ5+τ6)+τ7(τ7+τ8)
(τ5+τ6)2+(τ7+τ8)2,s6=−τ7(τ5+τ6)+τ5(τ7+τ8)
(τ5+τ6)2+(τ7+τ8)2,
s7=τ6(τ5+τ6)+τ8(τ7+τ8)
(τ5+τ6)2+(τ7+τ8)2,s8=−τ8(τ5+τ6)+τ6(τ7+τ8)
(τ5+τ6)2+(τ7+τ8)2,
s9=τ9(τ9+τ10)+τ11(τ11+τ12)
(τ9+τ10)2+(τ11+τ12)2,s10=−τ11(τ9+τ10)+τ9(τ11+τ12)
(τ9+τ10)2+(τ11+τ12)2,
s11=τ10(τ9+τ10)+τ12(τ11+τ12)
(τ9+τ10)2+(τ11+τ12)2,s12=−τ12(τ9+τ10)+τ10(τ11+τ12)
(τ9+τ10)2+(τ11+τ12)2,(45)
where
τ1=xj−xi
∥αi−αj∥2
2, τ2=xi−xk
∥αi−αk∥2
2, τ3=yj−yi
∥αi−αj∥2
2, τ4=yi−yk
∥αi−αk∥2
2,
τ5=yj−yi
∥βi−βj∥2
2, τ6=yi−yk
∥βi−βk∥2
2, τ7=zj−zi
∥βi−βj∥2
2, τ8=zi−zk
∥βi−βk∥2
2,
τ9=xj−xi
∥γi−γj∥2
2, τ10=xi−xk
∥γi−γk∥2
2, τ11=zj−zi
∥γi−γj∥2
2, τ12=zi−zk
∥γi−γk∥2
2.(46)
Algorithm 2Construction of Weight Matrices Hij,HikBased on
Plain Position
1:Available information: plain positions of agents i,j,k.
2:Ifαi̸ =αj̸ =αkdo
3:Calculating Hij,Hikby(49)–(54);
4:Else ifαi=αjdo
5:Calculating Hij,Hikby(55)–(58);
6:Else ifαi̸ =αj, αi=αkdo
7:Calculating Hij,Hikby(59)–(62);
8:Else, do
9:Calculating Hij,Hikby(63)–(69);
10:End
A.2. Calculation of weight matrices Hij,Hikbased on plain position
If follower i∈Vfand its two neighbors j,k∈Niare not
collocated, i.e., pi̸ =pj̸ =pk, there are four cases for the plain
9
X. Fang, L. Xie and D.V. Dimarogonas Automatica 175 (2025) 112188
positions αi, αj, αk: (1)αi̸ =αj̸ =αk; (2)αi=αj; (3)αi̸ =αj, αi=αk;
(4)αi̸ =αj,αj=αk.
Case (1): For the first caseαi̸ =αj̸ =αk, it has[
τ1τ3
−τ3τ1]
(αj−αi)+[
τ2τ4
−τ4τ2]
(αk−αi)=0,(47)
where τ1, τ2, τ3, τ4are given in(46). FromLemma A.1,
(τ1+τ2)2+(τ3+τ4)2̸ =0. (48)
For case (1), there are six subcases (1.1)-(1.6) for the plane
positions βi, βj, βkandγi, γj, γk: (1.1)βi=βj; (1.2)βi̸ =βj, βi=βk;
(1.3)βi̸ =βj̸ =βk; (1.4)βi̸ =βj, βj=βk, γi=γj; (1.5)βi̸ =βj, βj=
βk, γi̸ =γj, γi=γk; (1.6)βi̸ =βj, βj=βk, γi̸ =γj̸ =γk. Note that the
subcase βi̸ =βj, βj=βk, γj=γkis unreasonable if agents i,j,kare
not collocated. Other unreasonable subcases of case (2)-(4) will
also be excluded and not discussed.
(1.1) For the first subcase of case (1), it hasβi−βj=0. Then,
based on plain position relationship(47)and βi−βj=0, we can
obtain the weight matrices Hij,Hikin(6), i.e.,
Hij=[s1−s20
s21+s10
0 0 1]
,Hik=[s3−s40
s4 s30
0 0 0]
,(49)
where s1,s2,s3,s4are defined in(45). Then, we can calculate the
weight matrices Hij,Hikbased on the plain positions of agents
i,j,kfor the subcases (1.2)–(1.6).
(1.2) For the second subcase of case (1), the weight matrices
Hij,Hikare calculated as
Hij=[s1−s20
s2 s10
0 0 0]
,Hik=[s3−s40
s41+s30
0 0 1]
.(50)
(1.3) For the third subcase of case (1), the weight matrices Hij,
Hikare calculated as
Hij=[s1−s20
s2s1+s5−s6
0 s6 s5]
,Hik=[s3−s40
s4s3+s7−s8
0 s8 s7]
,(51)
where s5,s6,s7,s8are defined in(45).
(1.4) For the fourth subcase of case (1), the weight matrices
Hij,Hikare calculated as
Hij=[1+s1−s20
s2 s10
0 0 1]
,Hik=[s3−s40
s4 s30
0 0 0]
.(52)
(1.5) For the fifth subcase of case (1), the weight matrices Hij,
Hikare calculated as
Hij=[s1−s20
s2 s10
0 0 0]
,Hik=[1+s3−s40
s4 s30
0 0 1]
.(53)
(1.6) For the sixth subcase of case (1), the weight matrices Hij,
Hikare calculated as
Hij=[s1+s9−s2−s10
s2 s10
s100 s9]
,Hik=[s3+s11−s4−s12
s4 s30
s120 s11]
,(54)
where s9,s10,s11,s12are defined in(45).
Case (2): For the second caseαi=αj, it hasαi−αj=0. If the
agents i,j,kare not collocated, it hasβi̸ =βjandγi̸ =γj. There
are four subcases (2.1)-(2.4) for the plane positions βi, βj, βkand
γi, γj, γk: (2.1)βi=βk; (2.2)βi̸ =βj̸ =βk; (2.3)βi̸ =βk, βj=βk, γi=
γk; (2.4)βi̸ =βk, βj=βk, γi̸ =γj̸ =γk.
(2.1) For the first subcase of case (2), the weight matrices Hij,
Hikare calculated as
Hij=[1 0 0
0 1 0
0 0 0]
,Hik=[0 0 0
0 1 0
0 0 1]
. (55)(2.2) For the second subcase of case (2), the weight matrices
Hij,Hikare calculated as
Hij=[1 0 0
0 1+s5−s6
0 s6 s5]
,Hik=[0 0 0
0s7−s8
0s8 s7]
.(56)
(2.3) For the third subcase of case (2), the weight matrices Hij,
Hikare calculated as
Hij=[1 0 0
0 1 0
0 0 0]
,Hik=[1 0 0
0 0 0
0 0 1]
. (57)
(2.4) For the fourth subcase of case (2), the weight matrices
Hij,Hikare calculated as
Hij=[1+s90−s10
0 1 0
s100 s9]
,Hik=[s110−s12
0 0 0
s120 s11]
.(58)
Case (3): For the third caseαi̸ =αj, αi=αkof the plain
positions αi, αj, αk, it hasαi−αk=0. If the agents i,j,kare not
collocated, it hasβi̸ =βkandγi̸ =γk. There are four subcases
(3.1)-(3.4) for the plane positions βi, βj, βkandγi, γj, γk: (3.1)
βi=βj; (3.2) βi̸ =βj̸ =βk; (3.3) βi̸ =βj, βj=βk, γi=γj; (3.4)
βi̸ =βj, βj=βk, γi̸ =γj̸ =γk.
(3.1) For the first subcase of case (3),Hij,Hikare calculated as
Hij=[0 0 0
0 1 0
0 0 1]
,Hik=[1 0 0
0 1 0
0 0 0]
. (59)
(3.2) For the second subcase of case (3), the weight matrices
Hij,Hikare calculated as
Hij=[0 0 0
0s5−s6
0s6 s5]
,Hik=[1 0 0
0 1+s7−s8
0 s8 s7]
.(60)
(3.3) For the third subcase of case (3), the weight matrices
Hij,Hikare calculated as
Hij=[1 0 0
0 0 0
0 0 1]
,Hik=[1 0 0
0 1 0
0 0 0]
. (61)
(3.4) For the fourth subcase of case (3), the weight matrices
Hij,Hikare calculated as
Hij=[s90−s10
0 0 0
s100 s9]
,Hik=[1+s110−s12
0 1 0
s120 s11]
.(62)
Case (4): For the fourth caseαi̸ =αj, αj=αkof the plain
positions αi, αj, αk, it hasβj̸ =βkandγj̸ =γkif the agents i,j,k
are not collocated. There are seven subcases (4.1)-(4.7) for the
plane positions βi, βj, βkandγi, γj, γk: (4.1) βi=βj, γi=γk;
(4.2) βi=βj, γi̸ =γk; (4.3) βi̸ =βj, βi=βk, γi=γj; (4.4)
βi̸ =βj, βi=βk, γi̸ =γj; (4.5) βi̸ =βj̸ =βk, γi=γj; (4.6)
βi̸ =βj̸ =βk, γi̸ =γj, γi=γk; (4.7)βi̸ =βj̸ =βk, γi̸ =γj̸ =γk.
(4.1) For the first subcase of case (4), the weight matrices Hij,
Hikare calculated as
Hij=[0 0 0
0 1 0
0 0 1]
,Hik=[1 0 0
0 0 0
0 0 1]
. (63)
(4.2) For the second subcase of case (4), the weight matrices
Hij,Hikare calculated as
Hij=[s90−s10
0 1 0
s100 1+s9]
,Hik=[s110−s12
0 0 0
s120 s11]
.(64)
10
X. Fang, L. Xie and D.V. Dimarogonas Automatica 175 (2025) 112188
(4.3) For the third subcase of case (4), the weight matrices Hij,
Hikare calculated as
Hij=[1 0 0
0 0 0
0 0 1]
,Hik=[0 0 0
0 1 0
0 0 1]
. (65)
(4.4) For the fourth subcase of case (4), the weight matrices
Hij,Hikare calculated as
Hij=[s90−s10
0 0 0
s100s9]
,Hik=[s110−s12
0 1 0
s120 1+s11]
. (66)
(4.5) For the fifth subcase of case (4), the weight matrices Hij,
Hikare calculated as
Hij=[1 0 0
0s5−s6
0s61+s5]
,Hik=[0 0 0
0s7−s8
0s8 s7]
.(67)
(4.6) For the sixth subcase of case (4), the weight matrices Hij,
Hikare calculated as
Hij=[0 0 0
0s5−s6
0s6s5]
,Hik=[1 0 0
0s7−s8
0s81+s7]
. (68)
(4.7) For the seventh subcase of case (4), the weight matrices
Hij,Hikare calculated as
Hij=[s90−s10
0 s5−s6
s10 s6 s5+s9]
,Hik=[s110−s12
0 s7−s8
s12 s8 s7+s11]
.(69)
Appendix B. Proof ofLemma 3.1
Proof.If the agents i,j,kare not collocated, there are twenty-
one subcases for the calculation of the weight matrices Hij,Hikas
shown inAppendix A. For the first subcase(49), we have
Hii=Hij+Hik=[s1+s3−s2−s40
s2+s41+s1+s30
0 0 1]
. (70)
From(45), it hass1+s3=1 and s2+s4=0. Then,(70)becomes
Hii=Hij+Hik=[1 0 0
0 2 0
0 0 1]
. (71)
Hence, the matrix Hiiis nonsingular for the first subcase(49).
By using a similar argument, it can be shown that the matrix Hii
is nonsingular for the rest subcases(50)–(69).□
Algorithm 3Calculation of Translation-scaling Similar Configura-
tion
1:Available information: bearings or ratio-of-distances among
agents i,j,k.
2:Ifthe agents i,j,kare non-collinear and bearings are available
do
3: qi=0,qj=gij, and qkis calculated by(74);
4:End
5:Ifagents i,j,kare not collocated and both bearing and
ratio-of-distance measurements are available do
6: qi=0,qj=gij, and qkis calculated by(77);
7:End
Fig. 13.Examples of the distribution of the agents i,j,k.
Appendix C. Bearing-based translation-scaling similar config-
uration
If the agents i,j,kare non-collinear, we can obtain a
translation-scaling similar configuration ˇq= [qT
i,qT
j,qT
k]Tofˇp=
[pT
i,pT
j,pT
k]Tbased on bearing measurements shown as follows:
gij,gik, and gjkare the bearing measurements among the agents
i,j,k. If|gT
ijgik| ̸ =1, agents i,j,kare non-collinear shown in
Fig. 13(a). qiand qjare designed satisfying gij=qj−qi
∥qj−qi∥2. Set
qi= [0,0,0]Tand qj=gij. The distance dikbetween qiand qk
can be calculated by
dik= ∥qj−qi∥2·sinθijk
sinθikj=sinθijk
sinθikj, (72)
where the angles θijkandθikjare
θijk=arccos( −gT
ijgjk), θikj=arccos( gT
ikgjk). (73)
FromFig. 13(a), the position qkis calculated as
qk=dikgik+qi=giksin(arccos( −gT
ijgjk))
sin(arccos( gT
ikgjk)). (74)
For the positions qi,qj,qk, it has
qj−qi
∥qj−qi∥2=gij,qk−qj
∥qk−qj∥2=gjk,qk−qi
∥qk−qi∥2=gik.(75)
Hence, the configurations ˇq= [qT
i,qT
j,qT
k]Tandˇp= [pT
i,pT
j,pT
k]T
have the same inter-agent bearings, i.e., the configuration ˇq=
[qT
i,qT
j,qT
k]Tis a translation-scaling similar configuration ofˇp=
[pT
i,pT
j,pT
k]T. For example, if the positions of agents i,j,kare
pi= [1,0,1]T,pj= [3,0,1]T,pk= [2,2,3]T, we can calculate its
translation-scaling similar configuration ofˇp= [pT
i,pT
j,pT
k]Tas
qi= [0,0,0]T,qj= [1,0,0]T,qk= [1
2,1,1]T.(76)
Appendix D. Bearing-ratio-of-distance-based translation-
scaling similar configuration
If agents i,j,kare not collocated as shown inFig. 13(a)–(c).
We can combine bearing and ratio-of-distance measurements to
obtain a translation-scaling similar configuration ˇq= [qT
i,qT
j,qT
k]T
ofˇp= [pT
i,pT
j,pT
k]T. Note that rijk=∥pj−pi∥2
∥pk−pi∥2is the ratio-of-distance
measurement among agents pi,pj,pk. Setqi= [0,0,0]Tand qj=
gij. Then, it has
qk=gik∥qj−qi∥2
rijk=gik
rijk. (77)
For example, if the positions of agents i,j,karepi= [1,0,1]T,
pj= [3,0,1]T,pk= [4,0,1]T, we can calculate its translation-
scaling similar configuration ofˇp= [pT
i,pT
j,pT
k]Tas
qi= [0,0,0]T,qj= [1,0,0]T,qk= [3
2,0,0]T.(78)
11
X. Fang, L. Xie and D.V. Dimarogonas Automatica 175 (2025) 112188
