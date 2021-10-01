CREATE TABLE `solardata` (
  `id` bigint(20) NOT NULL AUTO_INCREMENT,
  `time` timestamp NULL DEFAULT current_timestamp(),
  `metric` varchar(25) DEFAULT NULL,
  `tag` varchar(25) DEFAULT NULL,
  `value` float DEFAULT NULL,
  `created` datetime DEFAULT current_timestamp(),
  `site` varchar(50) DEFAULT NULL,
  `device` varchar(50) DEFAULT NULL,
  `measure` varchar(50) DEFAULT NULL,
  PRIMARY KEY (`id`)
) ENGINE=InnoDB AUTO_INCREMENT=19401 DEFAULT CHARSET=utf8mb3 AVG_ROW_LENGTH=75;
