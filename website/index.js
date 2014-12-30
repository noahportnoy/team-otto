$(window).load(function(){
	var width = $(window).width();
	var height = $(window).height();
	
	$("#homeBox").css('left', ((width - $("#homeBox").width() ) / 2 )  + 'px');
	$("#options").css('left', ((width - $("#options").width() ) / 2 )  + 'px');
	
});

$(window).resize(function(){
	var width = $(window).width();
	var height = $(window).height();
	
	$("#homeBox").css('left', ((width - $("#homeBox").width() ) / 2 )  + 'px');
	$("#options").css('left', ((width - $("#options").width() ) / 2 )  + 'px');
	
});