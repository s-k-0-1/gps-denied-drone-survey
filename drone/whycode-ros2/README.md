## WhyCode / WhyCon

### A precise, efficient and low-cost localization system 

_WhyCon_ is a version of a vision-based localization system that can be used with low-cost web cameras, and achieves millimiter precision with very high performance.
The system is capable of efficient real-time detection and precise position estimation of several circular markers in a video stream. 
It can be used both off-line, as a source of ground-truth for robotics experiments, or on-line as a component of robotic systems that require real-time, precise position estimation.
_WhyCon_ is meant as an alternative to widely used and expensive localization systems.
It is fully open-source.

| WhyCon example application (video)  | Scenario description |
| ------ | ----------- |
|[![WhyCon applications](https://raw.githubusercontent.com/wiki/gestom/WhyCon/pics/whycon.png)](https://www.youtube.com/watch?v=KgKrN8_EmUA"AAAA")|-precise docking to a charging station (EU project STRANDS),<br/> -fitness evaluation for self-evolving robots (EU proj. SYMBRION),<br/>-relative localization of UAV-UGV formations (CZ-USA project COLOS),<br/>-energy source localization in (EU proj REPLICATOR),<br/>-robotic swarm localization (EU proj HAZCEPT).|

The _WhyCon_ system was developed as a joint project between the University of Buenos Aires (AR), the Czech Technical University in Prague (CZ) and the University of Lincoln (GB).
The main contributors were [Matias Nitsche](https://scholar.google.co.uk/citations?user=Z0hQoRUAAAAJ&hl=en&oi=ao), [Tom Krajnik](http://scholar.google.co.uk/citations?user=Qv3nqgsAAAAJ&hl=en&oi=ao), [Jan Faigl](https://scholar.google.co.uk/citations?user=-finD_sAAAAJ&hl=en) and [Peter Lightbody](https://scholar.google.com/citations?user=tBUM-8oAAAAJ&hl=cs&oi=ao).
The current main contributor and maintainer is [Jiří Ulrich](https://scholar.google.com/citations?hl=cs&user=vMtZ5FcAAAAJ).

#### Getting started

For useage and node interfaces description see the [docs](docs/)

-----

#### License

The code is avaiable only for non-commercial research and educational purposes only, see the LICENSE file for details. In any other case or when You are not sure about licensing, please, contact us!

If you use this localization system for your research, please don't forget to cite at least one relevant paper below.

#### Where is it described ?

WhyCon was first presented on International Conference on Advanced Robotics 2013, later in the Journal of Intelligent and Robotics Systems and finally at the Workshop on Open Source Aerial Robotics during the International Conference on Intelligent Robotic Systems, 2015.
Its early version was also presented at the International Conference of Robotics and Automation, 2013.
An extension of the system, which used a necklace code to add ID's to the tags, achieved a best paper award at the SAC 2017 conference.
Improved version with the full 6DOF estimation was presented in the Applied Computing Review, 2023.
If you decide to use this software for your research, please cite relevant papers provided below.

#### References

1. J. Ulrich et al.: **[Real time fiducial marker localisation system with full 6 DOF pose estimation](https://dl.acm.org/doi/abs/10.1145/3594264.3594266)**. ACM SIGAPP Applied Computing Review, 2023. [[bibtex](https://gist.github.com/jiriUlr/3f325488596932d784bcff4178c11478)].
1. J. Ulrich et al.: [Towards fast fiducial marker with full 6 DOF pose estimation](https://dl.acm.org/doi/abs/10.1145/3477314.3507043). Symposium on Applied Computing, 2022. [[bibtex](https://gist.github.com/jiriUlr/7d333e90c43e6b41c79e5150c7a59267)].
1. J. Ulrich: [Fiducial marker-based multiple camera localisation system](https://dspace.cvut.cz/bitstream/handle/10467/101526/F3-DP-2022-Ulrich-Jiri-main.pdf). Master's thesis. Czech Technical University in Prague, 2022. [[bibtex](https://gist.github.com/jiriUlr/e8d53c7edd6b14c824e67e60596a489f)].
1. K. Zampachu: [Visual analysis of beehive queen behaviour](https://dspace.cvut.cz/bitstream/handle/10467/101048/F3-BP-2022-Zampachu-Kristi-main.pdf). Bachelor's thesis. Czech Technical University in Prague, 2022. [[bibtex](https://gist.github.com/jiriUlr/eb08ee4b183c615e312ab2db767e9b18)].
1. J. Ulrich: [Fiducial Marker Detection for Vision-Based Mobile Robot Localisation](https://dspace.cvut.cz/bitstream/handle/10467/89879/F3-BP-2020-Ulrich-Jiri-main.pdf). Bachelor's thesis. Czech Technical University in Prague, 2020. [[bibtex](https://gist.github.com/jiriUlr/348d42b7a1cdd08b94953adedc50c5d7)].
1. P. Lightbody, T. Krajník et al.: An Efficient Visual Fiducial Localisation System. Applied Computing Review, 2017.
1. P. Lightbody, T. Krajník et al.: A versatile high-performance visual fiducial marker detection system with scalable identity encoding Symposium on Applied Computing, 2017.
1. M. Nitsche, T. Krajník et al.: WhyCon: An Efficent, Marker-based Localization System. IROS Workshop on Open Source Aerial Robotics, 2015.
1. T. Krajník, M. Nitsche et al.: A Practical Multirobot Localization System. Journal of Intelligent and Robotic Systems (JINT), 2014.
1. T. Krajník, M. Nitsche et al.: External localization system for mobile robotics. International Conference on Advanced Robotics (ICAR), 2013.
1. J. Faigl, T. Krajník et al.: Low-cost embedded system for relative localization in robotic swarms. International Conference on Robotics and Automation (ICRA), 2013.

#### Acknowledgements

The development of this work is currently supported by the EU FET Open programme under grant agreement No.964492 project _RoboRoyale_.
The development of this work was supported by the Czech Science Foundation project 17-27006Y _STRoLL_.
In the past, the work was supported by EU within its Seventh Framework Programme project ICT-600623 _STRANDS_.
The Czech Republic and Argentina have given support through projects 7AMB12AR022, ARC/11/11 and 13-18316P.
