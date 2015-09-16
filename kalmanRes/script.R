#!/usr/bin/env Rscript

library(e1071)
library(Rcmdr)

# gyro gyroEstimate angle(accelro) angleEstimate noise
data <- read.table("data.txt",skip=2,FALSE,sep=' ')

abs <- 1:length(data[,1])

png("AngularSpeed.png")
plot.new()
plot.window(xlim=c(min(abs),max(abs)),ylim=c(-20,20))
matplot(abs,cbind(data[,1],data[,2]),xlab="x",ylab="t",type="l",lty=1,main="Kalman AngularSpeed")
legend("bottomright", legend=cbind("Mesure","Estimate"), lty=1,col=1:2, cex = 0.75,bty="n")
dev.off()

png("AngleAccelro.png")
plot.new()
plot.window(xlim=c(min(abs),max(abs)),ylim=c(-20,20))
matplot(abs,cbind(data[,3],data[,4]),xlab="x",ylab="t",type="l",lty=1,main="Kalman AngleAccelero")
legend("bottomright", legend=cbind("Mesure","Estimate"), lty=1,col=1:2, cex = 0.75,bty="n")
dev.off()

png("noise.png")
plot.new()
plot.window(xlim=c(min(abs),max(abs)),ylim=c(-20,20))
matplot(abs,cbind(data[,5]),xlab="x",ylab="y",type="l",lty=1,main="Kalman Noise")
legend("bottomright", legend=cbind("Mesure"), lty=1,col=1:1, cex = 0.75,bty="n")
dev.off()

png("AngleGyro.png")
plot.new()
plot.window(xlim=c(min(abs),max(abs)),ylim=c(-20,20))
matplot(abs,cbind(data[,6]),xlab="x",ylab="y",type="l",lty=1,main="AngleGyro")
legend("bottomright", legend=cbind("Mesure"), lty=1,col=1:1, cex = 0.75,bty="n")
dev.off()
