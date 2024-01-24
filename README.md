## After Class Reading:

How to Animate 3D Characters in 1 Minute:
https://www.youtube.com/watch?v=TjJLIuFKA20

Human Body Rig in Blender
https://www.youtube.com/watch?v=MAM7mF2v7dE

Behind the Scenes God of War PS5 | Mocap Footage: 
https://www.youtube.com/watch?v=HVXoOK4R8M0

### Kinematics Animation

***IK***
- [x] 2017: http://www.andreasaristidou.com/publications/papers/IK_survey.pdf
- [ ] [Andreas Aristidou and Joan Lasenby. 2011 FABRIK: A fast, iterative solver for the Inverse Kinematics problem]
- [ ] [Ken Shoemake. 1985 Animating rotation with quaternion curves. SIGGRAPH Computer Graphics]


***Skinning***
- [ ] https://skinning.org/ Alec Jacobson, Zhigang Deng, Ladislav Kavan , and J. P. Lewis. Skinning: real time shape deformation. In ACM SIGGRAPH 2014 Courses (SIGGRAPH '14)
- [ ] [SMPL: A Skinned Multi Person Linear Model]
- [ ] [NeuroSkinning: Automatic Skin Binding for Production Characters with Deep Graph Networks Sig2019]
- [ ] [Learning Skeletal Articulations with Neural Blend Shapes Sig2021]
- [ ] https://rodolphe-vaillant.fr/entry/29/dual-quaternions-skinning-tutorial-and-c-codes
- [ ] https://ww2.mathworks.cn/help/matlab/ref/griddata.html
- [ ] J. P. Lewis, Matt Cordner , and Nickson Fong. 2000. Pose space deformation: a unified approach to shape interpolation and skeleton driven deformation . In Proceedings of the 27th annual conference on Computer graphics and interactive techniques (SIGGRAPH ’00), ACM Press/Addison Wesley Publishing Co., USA, 165 172.
- [ ] Dragomir Anguelov , Praveen Srinivasan, Daphne Koller, Sebastian Thrun , Jim Rodgers, and James Davis. 2005 . SCAPE: shape completion and animation of people . ACM Trans. Graph. 24 , 3 (July 2005 ), 408 416
- [ ] Egger et al. 2020. 3D Morphable Face Models Past, Present, and Future. ACM Trans. Graph. 39, 5 (June 2020), 157:1 157:38.


***Motion Retargeting*** 
- [ ] [Aberman et al. 2020 SIGGRAPH]
- [ ] [OpenPose, 2D Pose]
- [ ] [3D Video based Pose estimation, source: DeepMotion Inc.]
- [ ] [Holden 2018 Robust Solving of Optical Motion Capture Data by Denoising]
- [ ] https://captury.com/
- [ ] https://www.theiamarkerless.ca/
- [ ] [Yi et al. 2021. TransPose : Real time 3D Human Translation and Pose Estimation with Six Inertial Sensors]
- [ ] [Villegas et al. 2021, Contact Aware Retargeting of Skinned Motion]
- [x] nucl.ai Conference: Ubisoft Toronto "IK Rig" Prototype https://www.youtube.com/watch?v=V4TQSeUpH3Q

***Motion Graphs*** 
- [ ] [Lucas Kovar Frédéric Pighin . 2002. Motion graphs.]
- [ ] [Heck and Gleicher 2007, Parametric Motion Graphs]
- [ ] Motion Planning with Motion Graph and A* https://www.youtube.com/watch?v=ekx0bXz25Pw


***Motion Matching***  
- [ ] [Motion Field for Interactive Character Locomotion Sig2010]
- [x] https://www.gdcvault.com/play/1023280/Motion%20Matching%20and%20The%20Road



***Motion Blending*** 
- [ ] inertialized blending: https://wwww.theorangeduck.com/page/spring-roll-call by Daniel Holden 


***Learning based Approaches, Autoregressive Models*** 
traditional Gaussian Distribution:
- [ ] Synthesizing Physically Realistic Human Motion in Low-Dimensional Behavior-Specific Spaces, Sig2004
- [ ] Interactive Generation of Human Aniamtion with Deformable Motion Models, Sig2009
- [ ] Continuous Character Control with Low-Dimensional Embeddings Sig2012

Neural Networks:
- [ ] [Starke et al 2020, Local Motion Phases for Learning Multi Contact Character Movements]
- [ ] Pirker and Katzenschlager 2017. Gait disorders in adults and the elderly.


***Motion Generative Models*** 
- [ ] [Ling et al. 2021 Character Controllers Using Motion VAEs]
- [ ] [Henter et al. 2020, MoGlow : Probabilistic and Controllable Motion Synthesis Using Normalising Flows]
- [ ] [Zhang et al. 2022, arXiv , MotionDiffuse : Text Driven Human Motion Generation with Diffusion Model]
- [ ] [Tevet et al. 2022, arXiv , MDM: Human Motion Diffusion Model]

***Cross Modal Motion Synthesis*** 
- [ ] [Ao et al. 2022. Rhythmic Gesticulator. SIGGRAPH Asia 2022]
- [ ] [Tevet et al. 2022. MotionCLIP]


***Motion Reconstruction with Sparse Sensors*** 
- [ ] [DeepMotion: Virtual Reality Tracking]
- [ ] [Ye et al. 2022:]
- [ ] [Yang et al. 2022: Learning to Use Chopsticks]



### Physics Character Animation

***RigidBody***
- [ ] https://www.cs.cmu.edu/~baraff/sigcourse/
- [ ] David Baraff . SIGGRAPH ’94 Fast contact force computation for nonpenetrating rigid bodies.
- [ ] https://www.youtube.com/watch?v=WFKGWfdG3bU
- [ ] https://www.theorangeduck.com/page/spring-roll-call by Daniel Holden

***Spacetime/Trajectory Optimization*** 
- [ ] [Hodgins and Wooten 1995, Animating Human Athletics]
- [ ] [ControlVAE Yao et al. 2022]
- [ ] [Liu et al 2010. SAMCON]
- [ ] [Wampler and Popović . 2009. Optimal gait and form for animal locomotion]
- [ ] [AlBorno et al. 2013 Trajectory Optimization for Full Body Movements with Complex Contacts]
- [ ] [Hamalainen et al. 2020, Visualizing Movement Control Optimization Landscapes]
- [ ] [Zordan et al. 2005 Dynamic response for motion capture animation]
- [ ] [Stable Proportional-Derivative Controllers]
- [ ] [Macchietto et al. 2009 Momentum Control for Balance]


***Abstract Models***
- [ ] [Yin et al. 2007, SIMBICON: Simple Biped Locomotion Control]
- [ ] [Coros et al. 2010 Generalized Biped Walking Control Inverted Pendulum Model]
- [ ] [Biped Walking Pattern Generation by using Preview Control of Zero-Moment Point]
- [ ] [Muico et al 2011 Composite Control of Physically Simulated Characters]
- [ ] [Liu et al. 2012 Terrain Runner]


***DRL based Tracking Controllers***
- [ ] [Liu et al. 2016. ControlGraphs]
- [ ] [Liu et al. 2018]
- [ ] [Peng et al. 2018.DeepMimic]
- [ ] [Mnih et al. 2015, Human level control through deep reinforcement learning]

***Multi skill Characters***
- [ ] [Liu et al. 2017: Learning to Schedule Control Fragments ]


***Most of the basic C++ code can be found in Unreal Engine, you can find the summaries here: keneyr.com/Animation/***