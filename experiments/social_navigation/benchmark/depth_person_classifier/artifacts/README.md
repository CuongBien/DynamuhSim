# Selected pilot model release

Winner: depth_linear,30depth shape descriptors,31trainable parameters,threshold0.5. Trained on54train rows; selected from4fixed candidates by30validation rows; tested once on48rows (TP23/FN1/FP0/TN24). Same-avatar simulator pilot only. No H1–H5 or production semantic validity.

Copy this entire directory. `predict.Classifier().predict(sensor_sample, scan_geometry)` requires NumPy only. `sensor_sample` follows the portable corpus loader contract. It never reads labels, instance masks, world positions, future observations or ROS. `selected_model.npz` includes train-only normalization and fitted coefficients; `selected_model.pt` is the identical Torch state dictionary. Config/model SHA is checked before prediction.

This class does not generate proposals or track identities. Binary scores are not calibrated confidence; partial occlusion caused the retained false negative. The robot LiDAR remains in the project; the selected small classifier uses depth because fusion did not improve validation. See the parent report before applying outside this exact sensor/geometry profile.
