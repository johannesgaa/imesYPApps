package de.unihannover.imes.imesypmesse;


import android.app.Activity;
import android.content.Context;
import android.content.Intent;
import android.content.res.TypedArray;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.os.Bundle;
import android.view.View;
import android.view.ViewGroup;
import android.widget.AdapterView;
import android.widget.AdapterView.OnItemLongClickListener;
import android.widget.BaseAdapter;
import android.widget.Gallery;
import android.widget.ImageView;

public class MainActivity extends Activity
{
	//variable for selection intent
  	private final int PICKER = 1;
	//variable to store the currently selected image
	private int currentPic = 0;
	//gallery object
	@SuppressWarnings("deprecation")
	private Gallery picGallery;
	//image view for larger display
	private ImageView picView;
	
	//adapter for gallery view
	//private PicAdapter imgAdapt;
	
    /** Called when the activity is first created. */
    @Override
    public void onCreate(Bundle savedInstanceState)
    {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.main);
        
      //get the large image view
     //   picView = (ImageView) findViewById(R.id.picture);
        //get the gallery view
      //  picGallery = (Gallery) findViewById(R.id.gallery1);
        
      //create a new adapter
      //  imgAdapt = new PicAdapter(this);
        //set the gallery adapter
       // picGallery.setAdapter(imgAdapt);
      //set long click listener for each gallery thumbnail item
       // picGallery.setOnItemLongClickListener(new OnItemLongClickListener() {
            //handle long clicks
        //    public boolean onItemLongClick(AdapterView<?> parent, View v, int position, long id) {
                //take user to choose an image
            	//update the currently selected position so that we assign the imported bitmap to correct item
          //  	currentPic = position;
            	//start rigth demoactivity
            	//Intent intent = new Intent(this);
            	//intent.putExtra(EXTRA_MESSAGE, message);
                //startActivity(intent);

            	
            	
            	
           // 	return true;
            	
         //   }
       //});

        
        
    }
   
    	
    	
    }

