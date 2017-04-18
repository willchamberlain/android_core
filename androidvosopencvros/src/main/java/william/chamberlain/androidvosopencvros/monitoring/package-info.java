/**
 * Entities and functions for extracting data from robots, smart cameras, routers, queues,
 * computation units, QoS monitors, and other system entities - data used for system-external
 * functions at debugging, the operations console, demonstrations.
 *
 * Includes, for example,
 *
 * - publishing poses, transforms, and covariances into TF
 *
 * - _manual_ system health monitoring - _automated_ monitoring is a system function and packaged at the application level
 *
 * - aggregating and publishing joint data from separate cameras ; can't think of a specific example, but sounds reasonable for demo purposes
 *
 *   - e.g.  https://www.fastcodesign.com/3022701/new-mit-media-lab-tool-lets-anyone-visualize-unwieldy-government-data
 *
 *   - https://books.google.com.au/books?id=aAihCgAAQBAJ&pg=PA92&lpg=PA92&dq=visualise+robot+data&source=bl&ots=cGnJzz0JYN&sig=PMS3Rc4ty81xAWyZH-x6RVrSREo&hl=en&sa=X&ved=0ahUKEwifqrmJlenSAhUKipQKHaAMCQYQ6AEISjAK#v=onepage&q=visualise%20robot%20data&f=false
 *
 *   - https://github.com/swri-robotics/mapviz
 *
 * The package structure is oriented to robotic vision, so generic vision system entities functions
 * like resolution and exposure, and image-processing and robotic-embodiment related logic like
 * geometry transform handling, are retained at the application level, rather than attempting to
 * develop a rationale for packaging.
 */