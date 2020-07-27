---
title: Receiver Operating Characteristic (ROC)
date: 27 July 2020
---

<style type="text/css" rel="stylesheet">
/* Three image containers (use 25% for four, and 50% for two, etc) */
.column {
  float: left;
  width: 33.33%;
  padding: 5px;
}

/* Clear floats after image containers */
.row::after {
  content: "";
  clear: both;
  display: table;
}
</style>


## Terms

$$
TPR = Recall = Sensitivity = \frac{TP}{TP+FN} \\
Specificity = \frac{TN}{TN + FP} \\
FPR = 1 - Specificity = \frac{FP}{TN + FP} \\
Accuracy = \frac{TP + TN}{P + N} \\
Precision = \frac{TP}{TP + FP}
$$

## Relations

<div class="row">
  <div class="column">
    <img src="a.png" alt="Snow" style="width:100%">
  </div>
  <div class="column">
    <img src="aa.png" alt="Forest" style="width:100%">
  </div>
</div>


<div class="row">
  <div class="column">
    <img src="b.png" alt="Snow" style="width:100%">
  </div>
  <div class="column">
    <img src="bb.png" alt="Forest" style="width:100%">
  </div>
</div>


<div class="row">
  <div class="column">
    <img src="c.png" alt="Snow" style="width:100%">
  </div>
  <div class="column">
    <img src="cc.png" alt="Forest" style="width:100%">
  </div>
</div>


<div class="row">
  <div class="column">
    <img src="d.png" alt="Snow" style="width:100%">
  </div>
  <div class="column">
    <img src="dd.png" alt="Forest" style="width:100%">
  </div>
</div>

## References

- [AUC - ROC Curve](https://towardsdatascience.com/understanding-auc-roc-curve-68b2303cc9c5)
