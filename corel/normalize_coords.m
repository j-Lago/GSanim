pts = [174, 93
    ]

norm_pts = (pts-350)/350
plot(norm_pts(1:2:end), norm_pts(2:2:end), 'o')
dlmwrite('asset.csv', norm_pts)
clipboard('copy', mat2str(norm_pts))


