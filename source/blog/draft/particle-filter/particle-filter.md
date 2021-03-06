---
title: Particle Filter Overview
date: xxx
---
 
- Particle filter
    - Work well on nonlinear and non-Gaussian systems
    - Multi-modal processing capability
    - Based on Monte Carlo methods which use particles to represent probabilities associated with any state space
    - Initially, the particles are spread over the state space randomly with random probabilities
    - As the system progresses in time, the particles converge
    - Particle filters able to do global search across a state space
    - Ideally, as the number of particles approaches infinity, they can model any nonlinear system
    - Process
        - Initialization: random distribution across state space
        - Loop:
            - Predictions
            - Update
            - Resample
    - Pros:
        - Simple to implement
        - Low memory requirement
        - Multi-hypothesis
        - Can use adaptive computation by adjusting sigma depending on the available resources (cpu or time)
        - Can adaptively converge by changing the decay factor (lambda) depending on the precision and time
    - Cons:
        - Typically need a large number of samples (particles)
        - Resampling (when the number of particles drops below a threshold, density of particles doesn’t represent the pdf, etc), can result in the loss of sample validity and diversity
            - Particle depletion: one (or few) particle(s) with a very high probability and lots of particles with essentially a nil (0.000000001) probability
        - No strict proof of convergences, might get caught in a local minimum

- Kalman Filters
    - Assume Gaussian noise in systems
    - Need a  Bayesian model of  a system, generally, this model is a simplified model of a complex system and it is good enough
    - Works on linear systems or nonlinear systems you can linearize at each time step (EKF)
- Unscented Kalman Filter (UKF)
    - Similar to the EKF but no linearization
    - Only use a few sample points call Sigma points
- Unscented particle filter
    - Uses an UKF to generate the importance distribution, which is larger than the overlap of the real state PDF and the estimation accuracy is higher
    
# References

- [Particle filter for data science](https://towardsdatascience.com/particle-filter-a-hero-in-the-world-of-non-linearity-and-non-gaussian-6d8947f4a3dc)
- MIT: [Particle Filters](https://web.mit.edu/16.412j/www/html/Advanced%20lectures/Slides/Hsaio_plinval_miller_ParticleFiltersPrint.pdf)
