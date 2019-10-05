Sequences and Series
=====================

:date: 2015-10-11
:summary: Infinite series summary

Sequence
---------

A **sequence** is an ordered list of numbers (e.g., :math:`a_n`); the numbers are called
"elements" or "terms". Every convergent sequence is bounded, thus an unbounded sequence
is divergent.

================== =========================================================================================================================== =========
Sequence Test      Converge                                                                                                                     Notes
================== =========================================================================================================================== =========
Squeeze Theorem    :math:`\lim\limits_{n \to \infty} a_n = \lim\limits_{n \to \infty} c_n = L` then :math:`\lim\limits_{n \to \infty} b_n = L` :math:`a_n \le b_n \le c_n`
Def 1, pg 692      :math:`\lim\limits_{n \to \infty} a_n = L`
l'Hospital's Rule  :math:`\lim\limits_{n \to \infty} \frac{f(x)}{g(x)} \Rightarrow \lim\limits_{n \to \infty} \frac{f'(x)}{g'(x)}`              where :math:`f(x)` = numerator and :math:`g(x)` = denominator
Theorem 3, pg 693  if :math:`f(n) = a_n` then :math:`\lim\limits_{n \to \infty} f(x)=L`
Theorem 6, pg 694  :math:`\lim\limits_{n \to \infty} | a_n | =0` then :math:`a_n` converges
Theorem 9, pg 696  :math:`\lim\limits_{n \to \infty} r^n = \begin{cases} 0, & \text{if } -1 < r < 1 \\ 1, & \text{if } r = 1 \end{cases}`      Divergent for all other values of :math:`r`
Theorem 12, pg 698 Every bounded (:math:`m \le a_n \le M`), monotonic sequence is convergent                                                   The bounds exists for :math:`n \ge 1`, also see Theorem 10 and 11
================== =========================================================================================================================== =========

Series
------

A **series** is the sum of the terms of a sequence: :math:`\sum\limits_{n=1}^\infty a_n`.


==================== =========================================================================================================== ========================================================================================================= =================================================================================
 Series Test         Converge                                                                                                    Diverge                                                                                                    Notes
==================== =========================================================================================================== ========================================================================================================= =================================================================================
 Divergence          N/A                                                                                                         :math:`\lim\limits_{n\to\infty} a_n \ne 0`                                                                 Doesn't show convergence and the converse is not true
 Integral            if :math:`\int\limits_1^\infty f(x) dx` converges                                                           if :math:`\int\limits_1^\infty f(x) dx` diverges                                                           :math:`f(x)` must be positive, decreasing, and continous, also :math:`f(n) = a_n \text{ for all } n`
 Root                :math:`\lim\limits_{n\to\infty}\sqrt[n]{|a_n|} = L < 1`                                                     :math:`\lim\limits_{n\to\infty}\sqrt[n]{|a_n|} = L > 1 \text{ or } \infty`                                 inconclusive if :math:`L = 1`
 Ratio               :math:`\lim\limits_{n\to\infty} \left| \frac{a_{n+1}}{a_n}\right| = L < 1`                                  :math:`\lim\limits_{n\to\infty} \left| \frac{a_{n+1}}{a_n}\right| = L > 1 \text{ or } \infty`              inconclusive if :math:`L = 1`
 Direct Comparison   :math:`0 \le a_n \le b_n  \text{ for all } n` and :math:`\sum\limits_{n=1}^{\infty} b_n` converges          :math:`0 \le b_n \le a_n  \text{ for all } n` and :math:`\sum\limits_{n=1}^{\infty} b_n` diverges          :math:`a_n,b_n > 0`
 Limit Comparison    :math:`\lim\limits_{n\to\infty} \frac{a_n}{b_n} = L` and :math:`\sum\limits_{n=1}^{\infty} b_n` converges   :math:`\lim\limits_{n\to\infty} \frac{a_n}{b_n} = L` and :math:`\sum\limits_{n=1}^{\infty} b_n` diverges   :math:`a_n,b_n > 0` and L is a positive constant, if L is :math:`\infty` or 0, then pick a different :math:`b_n`
 Absolute            :math:`\sum\limits_{n=1}^{\infty} | a_n | = 0`                                                                                                                                                                         Definition of absolutely convergent, the sum is independent of the order in which the terms are summed
 Conditional         :math:`\sum\limits_{n=1}^{\infty} | a_n |` diverges but :math:`\sum\limits_{n=1}^{\infty} a_n` converges                                                                                                               The sum is dependent of the order in which the terms are summed
==================== =========================================================================================================== ========================================================================================================= =================================================================================

Common Series
-------------

==================== ============================================================ ========================================================================================================================================== ================================================================================================================================ =================================================================================
 Series Test         Formula                                                      Converge                                                                                                                                    Diverge                                                                                                                         Notes
==================== ============================================================ ========================================================================================================================================== ================================================================================================================================ =================================================================================
 Alternating         :math:`\sum\limits_{n=1}^\infty (-1)^{n-1} a_n`              :math:`0 < a_{n+1} \le a_n \text{ for all } n` and :math:`\lim\limits_{n \to \infty} a_n = 0`                                              N/A
 Geometric           :math:`\sum\limits_{n=1}^\infty ar^{n-1}`                    :math:`|r| < 1` and converges to :math:`\frac{a}{1-r}`                                                                                     :math:`|r| \ge 1`                                                                                                                finite sum of the first n terms: :math:`= \frac{a(1-r^n)}{1-r}`
 P-Series            :math:`\sum\limits_{n=1}^\infty \frac{1}{n^p}`               :math:`p > 1`                                                                                                                              :math:`p \le 1`                                                                                                                  cannot calculate sum
 Power               :math:`\sum\limits_{n=0}^\infty c_n (x-a)^n`                 :math:`\begin{array}{l} i,  \text{converge if } x=a \\  ii,  \text{converge for all } x \\ iii,  \text{converge if } |x-a|<R \end{array}`                                                                                                                                   :math:`R` is the radius of convergence, you need to check the end points for convergence too. Typically use Ratio Test.
 Taylor              :math:`\sum\limits_{n=0}^\infty \frac{f^n (a)}{n!} (x-a)^n`  :math:`|x-a|<R`                                                                                                                            :math:`|x-a|>R`                                                                                                                  Taylor series is centered about a. Same note as power series
 Maclaurin           :math:`\sum\limits_{n=0}^\infty \frac{f^n (0)}{n!} (x)^n`    :math:`|x|<R`                                                                                                                              :math:`|x|>R`                                                                                                                    A Macluarin series is a Taylor series centered about 0. Same note as power series
==================== ============================================================ ========================================================================================================================================== ================================================================================================================================ =================================================================================
